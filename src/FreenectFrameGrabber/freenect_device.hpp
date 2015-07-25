#ifndef FREENECT_DEVICE_T01IELX0
#define FREENECT_DEVICE_T01IELX0

// copied from: https://github.com/piyushk/freenect_stack

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/thread/mutex.hpp>
#include <stdexcept>

#include <libfreenect.h>
#include <libfreenect_registration.h>
#include "image_buffer.hpp"

namespace freenect_camera {

  static const std::string PRODUCT_NAME = "Xbox NUI Camera";
  static const unsigned PRODUCT_ID = 0x2ae;
  static const std::string VENDOR_NAME = "Microsoft";
  static const unsigned VENDOR_ID = 0x45e;

  typedef freenect_resolution OutputMode;

  bool isImageMode(const ImageBuffer& buffer) {
    return buffer.metadata.video_format == FREENECT_VIDEO_RGB;
  }

  class FreenectDriver;

  class FreenectDevice : public boost::noncopyable {

    public:

      FreenectDevice(freenect_context* driver, std::string serial) {
        if (freenect_open_device_by_camera_serial(driver, &device_, serial.c_str()) < 0) {
          throw std::runtime_error("[ERROR] Unable to open specified kinect");
        }
        freenect_set_user(device_, this);
        freenect_set_depth_callback(device_, freenectDepthCallback);
        freenect_set_video_callback(device_, freenectVideoCallback);

        // turn of any dynamic correction
        freenect_set_flag(device_, FREENECT_AUTO_EXPOSURE, FREENECT_ON);
        freenect_set_flag(device_, FREENECT_AUTO_WHITE_BALANCE, FREENECT_OFF);

        device_serial_ = serial;
        registration_ = freenect_copy_registration(device_);

        //Initialize default variables
        streaming_video_ = should_stream_video_ = false;
        new_video_resolution_ = getDefaultImageMode();
        new_video_format_ = FREENECT_VIDEO_RGB;
        video_buffer_.metadata.resolution = FREENECT_RESOLUTION_DUMMY;
        video_buffer_.metadata.video_format = FREENECT_VIDEO_DUMMY;

        streaming_depth_ = should_stream_depth_ = false;
        new_depth_resolution_ = getDefaultDepthMode();
        new_depth_format_ = FREENECT_DEPTH_MM;
        depth_buffer_.metadata.resolution = FREENECT_RESOLUTION_DUMMY;
        depth_buffer_.metadata.depth_format = FREENECT_DEPTH_DUMMY;
      }

      ~FreenectDevice() {
        shutdown();
      }

      void shutdown() {
        freenect_close_device(device_);
        freenect_destroy_registration(&registration_);
      }

      /* DEVICE SPECIFIC FUNCTIONS */

      /** Unsupported */
      unsigned getBus() const {
        return 0;
      }

      /** Unsupported */
      unsigned getAddress() const {
        return 0;
      }

      const char* getProductName() const {
        return PRODUCT_NAME.c_str();
      }

      const char* getSerialNumber() const {
        return device_serial_.c_str();
      }

      bool hasImageStream() const {
        return true;
      }

      bool hasDepthStream() const {
        return true;
      }

      bool hasIRStream() const {
        return true;
      }

      bool isDepthRegistrationSupported() const {
        return true;
      }

      bool isSynchronizationSupported() const {
        return false;
      }

      bool isSynchronized() const {
        return false;
      }

      bool setSynchronization(bool on_off) const {
        throw std::runtime_error("[ERROR] The kinect does not support hardware synchronization");
      }

      /**
       * Get the baseline (distance between rgb/depth sensor)
       */
      float getBaseline() const {
        return 0.01 * registration_.zero_plane_info.dcmos_emitter_dist;
      }

      /* CALLBACK ASSIGNMENT FUNCTIONS */

      template<typename T> void registerImageCallback (
          void (T::*callback)(const ImageBuffer& image, void* cookie), 
          T& instance, void* cookie = NULL) {
        image_callback_ = boost::bind(callback, boost::ref(instance), _1, cookie);
      }

      template<typename T> void registerDepthCallback (
          void (T::*callback)(const ImageBuffer& depth_image, void* cookie), 
          T& instance, void* cookie = NULL) {
        depth_callback_ = boost::bind(callback, boost::ref(instance), _1, cookie);
      }

      template<typename T> void registerIRCallback (
          void (T::*callback)(const ImageBuffer& ir_image, void* cookie), 
          T& instance, void* cookie = NULL) {
        ir_callback_ = boost::bind(callback, boost::ref(instance), _1, cookie);
      }

      /* IMAGE SETTINGS FUNCTIONS */

      OutputMode getImageOutputMode() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        return video_buffer_.metadata.resolution;
      }

      void setImageOutputMode(OutputMode mode) {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        new_video_resolution_ = mode;
      }

      OutputMode getDefaultImageMode() const {
        return FREENECT_RESOLUTION_MEDIUM;
      }

      bool findCompatibleImageMode(const OutputMode& mode, OutputMode& compatible_mode) const {
        freenect_frame_mode new_mode = 
          freenect_find_video_mode(mode, video_buffer_.metadata.video_format);
        if (!new_mode.is_valid) {
          compatible_mode = getDefaultImageMode();
          return false;
        }
        compatible_mode = mode;
        return true;
      }

      void stopImageStream() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        should_stream_video_ = 
          (isImageStreamRunning()) ? false : streaming_video_;
      }

      void startImageStream() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        new_video_format_ = FREENECT_VIDEO_RGB;
        should_stream_video_ = true;
      }

      bool isImageStreamRunning() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        return streaming_video_ && _isImageModeEnabled();
      }

      void stopIRStream() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        should_stream_video_ = 
          (isIRStreamRunning()) ? false : streaming_video_;
      }

      void startIRStream() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        new_video_format_ = FREENECT_VIDEO_IR_8BIT;
        should_stream_video_ = true;
      }

      bool isIRStreamRunning() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        return streaming_video_ && !_isImageModeEnabled();
      }

      /* DEPTH SETTINGS FUNCTIONS */

      OutputMode getDepthOutputMode() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        return depth_buffer_.metadata.resolution;
      }

      void setDepthOutputMode(OutputMode mode) {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        new_depth_resolution_ = mode;
      }

      OutputMode getDefaultDepthMode() const {
        return FREENECT_RESOLUTION_MEDIUM;
      }

      bool findCompatibleDepthMode(const OutputMode& mode, OutputMode& compatible_mode) const {
        freenect_frame_mode new_mode = 
          freenect_find_depth_mode(mode, depth_buffer_.metadata.depth_format);
        if (!new_mode.is_valid) {
          compatible_mode = getDefaultDepthMode();
          return false;
        }
        compatible_mode = mode;
        return true;
      }

      bool isDepthRegistered() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        return depth_buffer_.metadata.depth_format == FREENECT_DEPTH_REGISTERED;
      }

      void setDepthRegistration(bool enable) {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        new_depth_format_ = 
          (enable) ? FREENECT_DEPTH_REGISTERED : FREENECT_DEPTH_MM;
      }

      void stopDepthStream() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        should_stream_depth_ = false;
      }

      void startDepthStream() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        should_stream_depth_ = true;
      }

      bool isDepthStreamRunning() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        return streaming_depth_;
      }

      /* LIBFREENECT ASYNC CALLBACKS */

      static void freenectDepthCallback(
          freenect_device *dev, void *depth, uint32_t timestamp) {

        FreenectDevice* device = 
            static_cast<FreenectDevice*>(freenect_get_user(dev));
        device->depthCallback(depth);
      }

      static void freenectVideoCallback(
          freenect_device *dev, void *video, uint32_t timestamp) {

        FreenectDevice* device = 
            static_cast<FreenectDevice*>(freenect_get_user(dev));
        device->videoCallback(video);
      }

    private:


      freenect_device* device_;
      std::string device_serial_;
      freenect_registration registration_;

      boost::function<void(const ImageBuffer&)> image_callback_;
      boost::function<void(const ImageBuffer&)> depth_callback_;
      boost::function<void(const ImageBuffer&)> ir_callback_;

      ImageBuffer video_buffer_;
      bool streaming_video_;
      bool should_stream_video_;
      freenect_resolution new_video_resolution_;
      freenect_video_format new_video_format_; 

      ImageBuffer depth_buffer_;
      bool streaming_depth_;
      bool should_stream_depth_;
      freenect_resolution new_depth_resolution_;
      freenect_depth_format new_depth_format_;

      /* Prevents changing settings unless the freenect thread in the driver
       * is ready */
      boost::recursive_mutex m_settings_;

    bool _isImageModeEnabled() {
      boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
      return isImageMode(video_buffer_);
    }

    void depthCallback(void* depth) {
      boost::lock_guard<boost::mutex> buffer_lock(depth_buffer_.mutex);
      assert(depth == depth_buffer_.image_buffer.get());
      depth_callback_.operator()(depth_buffer_);
    }

    void videoCallback(void* video) {
      boost::lock_guard<boost::mutex> buffer_lock(video_buffer_.mutex);
      assert(video == video_buffer_.image_buffer.get());
      if (isImageMode(video_buffer_)) {
        image_callback_.operator()(video_buffer_);
      } else {
        ir_callback_.operator()(video_buffer_);
      }
    }


  public:
      void executeChanges() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);

        bool change_video_settings = 
          video_buffer_.metadata.video_format != new_video_format_ ||
          video_buffer_.metadata.resolution != new_video_resolution_ ||
          streaming_video_ != should_stream_video_;

        if (change_video_settings) {
          // Stop video stream
          freenect_stop_video(device_);
          streaming_video_ = false;
          // Allocate buffer for video if settings have changed
          if (video_buffer_.metadata.resolution != new_video_resolution_ ||
              video_buffer_.metadata.video_format != new_video_format_) {
            try {
              allocateBufferVideo(video_buffer_, new_video_format_, 
                 new_video_resolution_, registration_);
            } catch (std::runtime_error& e) {
              printf("[ERROR] Unsupported video format/resolution provided. %s\n",
                  e.what());
              printf("[INFO] Setting default settings (RGB/VGA)\n");
              allocateBufferVideo(video_buffer_, FREENECT_VIDEO_BAYER,
                  FREENECT_RESOLUTION_MEDIUM, registration_);
            }
            freenect_set_video_mode(device_, video_buffer_.metadata);
            freenect_set_video_buffer(device_, video_buffer_.image_buffer.get());
            new_video_resolution_ = video_buffer_.metadata.resolution;
            new_video_format_ = video_buffer_.metadata.video_format;
          }
          // Restart stream if required
          if (should_stream_video_) {
            freenect_start_video(device_);
            streaming_video_ = true;
          }
        }

        bool change_depth_settings = 
          depth_buffer_.metadata.depth_format != new_depth_format_ ||
          depth_buffer_.metadata.resolution != new_depth_resolution_ ||
          streaming_depth_ != should_stream_depth_;

        if (change_depth_settings) {
          // Stop depth stream
          freenect_stop_depth(device_);
          streaming_depth_ = false;
          // Allocate buffer for depth if settings have changed
          if (depth_buffer_.metadata.resolution != new_depth_resolution_ ||
              depth_buffer_.metadata.depth_format != new_depth_format_) {
            try {
              allocateBufferDepth(depth_buffer_, new_depth_format_, 
                 new_depth_resolution_, registration_);
            } catch (std::runtime_error& e) {
              printf("[ERROR] Unsupported depth format/resolution provided. %s\n",
                  e.what());
              printf("[INFO] Setting default settings (depth registered/VGA)\n");
              allocateBufferDepth(depth_buffer_, FREENECT_DEPTH_MM,
                  FREENECT_RESOLUTION_MEDIUM, registration_);
            }
            freenect_set_depth_mode(device_, depth_buffer_.metadata);
            freenect_set_depth_buffer(device_, depth_buffer_.image_buffer.get());
            new_depth_resolution_ = depth_buffer_.metadata.resolution;
            new_depth_format_ = depth_buffer_.metadata.depth_format;
          }
          // Restart stream if required
          if (should_stream_depth_) {
            freenect_start_depth(device_);
            streaming_depth_ = true;
          }
        }
      }

  };
}

#endif /* end of include guard: FREENECT_DEVICE_T01IELX0 */