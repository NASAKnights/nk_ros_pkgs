#include "v4l2_camera/v4l2_camera_node.hpp"
#include "v4l2_camera/v4l2_calibration.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

V4L2CameraNode::V4L2CameraNode()
    : Node("v4l2_camera_node")
{
    this->declare_parameter("device_id", "");
    this->declare_parameter("width", 0);  // default to 0 → auto
    this->declare_parameter("height", 0); // default to 0 → auto
    this->declare_parameter("calibration_base_path", "/.ros/calibration");

    _device_id = this->get_parameter("device_id").as_string();

    _device_path = resolve_device_id();

    _width  = this->get_parameter("width").as_int();
    _height = this->get_parameter("height").as_int();
    int fps = 0;

    if(_width <= 0 || _height <= 0)
    {
        if(get_max_resolution_and_fps(_device_path, _width, _height, fps) != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to determine max resolution for device: %s",
                         _device_path.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Using max resolution: %dx%d", _width, _height);
    }

    _publisher       = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    _camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    _calib_path = ament_index_cpp::get_package_share_directory("v4l2_camera") + "/config/" +
                  "calib_" + _serial_number + ".yaml";
    load_camera_info(_device_id);
    open_device();
    _timer = this->create_wall_timer(1000ms / fps, std::bind(&V4L2CameraNode::capture_loop, this));

    _calibration_server = std::make_shared<CalibrationHandler>(this);
}

int V4L2CameraNode::get_max_resolution_and_fps(const std::string& device_path, int& width,
                                               int& height, int& fps)
{
    RCLCPP_INFO(this->get_logger(), "Attempting to determine max resolution and FPS for device: %s",
                device_path.c_str());

    int fd = open(device_path.c_str(), O_RDWR);
    if(fd < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open device: %s", device_path.c_str());
        return -1;
    }

    struct v4l2_fmtdesc fmt = {};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.index               = 0;

    int max_score = 0;
    while(ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Found format: %s",
                    reinterpret_cast<char*>(&fmt.description));

        struct v4l2_frmsizeenum size = {};
        size.pixel_format            = fmt.pixelformat;
        size.index                   = 0;
        while(ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0)
        {
            if(size.type == V4L2_FRMSIZE_TYPE_DISCRETE)
            {
                RCLCPP_INFO(this->get_logger(), "Found resolution: %dx%d", size.discrete.width,
                            size.discrete.height);

                struct v4l2_frmivalenum ival = {};
                ival.pixel_format            = fmt.pixelformat;
                ival.width                   = size.discrete.width;
                ival.height                  = size.discrete.height;
                ival.index                   = 0;
                while(ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &ival) == 0)
                {
                    if(ival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
                    {
                        int area        = size.discrete.width * size.discrete.height;
                        int current_fps = ival.discrete.denominator / ival.discrete.numerator;
                        int score       = area * current_fps;
                        RCLCPP_INFO(this->get_logger(), "Found FPS: %d, Score: %d", current_fps,
                                    score);

                        if(score > max_score)
                        {
                            max_score = score;
                            width     = size.discrete.width;
                            height    = size.discrete.height;
                            fps       = current_fps;
                        }
                    }
                    ival.index++;
                }
            }
            size.index++;
        }
        fmt.index++;
    }

    close(fd);

    if(max_score > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Max resolution determined: %dx%d at %d FPS", width, height,
                    fps);
        return 0;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to determine max resolution and FPS for device: %s",
                     device_path.c_str());
        return -1;
    }
}

V4L2CameraNode::~V4L2CameraNode()
{
    stop_capture();
}

std::string V4L2CameraNode::resolve_device_id()
{
    RCLCPP_INFO(get_logger(), "device_id: '%s', serial_number: %s, ", _device_id.c_str(),
                _serial_number.c_str());
    // Prefer resolving by exact serial number
    if(!_serial_number.empty())
    {
        struct udev* udev = udev_new();
        if(!udev)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create udev context");
            return "";
        }

        struct udev_enumerate* enumerate = udev_enumerate_new(udev);
        udev_enumerate_add_match_subsystem(enumerate, "video4linux");
        udev_enumerate_scan_devices(enumerate);

        struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
        struct udev_list_entry* entry;

        udev_list_entry_foreach(entry, devices)
        {
            const char*         syspath = udev_list_entry_get_name(entry);
            struct udev_device* dev     = udev_device_new_from_syspath(udev, syspath);
            if(!dev)
                continue;

            struct udev_device* parent =
                udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
            if(!parent)
            {
                udev_device_unref(dev);
                continue;
            }

            const char* serial = udev_device_get_property_value(parent, "ID_SERIAL_SHORT");
            if(serial && _serial_number == serial)
            {
                const char* devnode = udev_device_get_devnode(dev); // e.g., "/dev/video2"
                if(devnode)
                {
                    _device_id = std::filesystem::path(devnode).filename().string();
                    RCLCPP_INFO(get_logger(), "Resolved device by serial '%s' to %s",
                                _serial_number.c_str(), devnode);

                    udev_device_unref(dev);
                    udev_enumerate_unref(enumerate);
                    udev_unref(udev);
                    return std::string(devnode);
                }
            }

            udev_device_unref(dev);
        }

        udev_enumerate_unref(enumerate);
        udev_unref(udev);
    }

    if(!_device_id.empty())
    {
        // Fall back to partial modalias match if no serial match found
        for(const auto& entry : fs::directory_iterator("/sys/class/video4linux"))
        {
            std::string dev_name   = entry.path().filename();
            std::string device_dir = entry.path().string() + "/device";
            std::string full_path  = "/dev/" + dev_name;
            RCLCPP_INFO(get_logger(), "Resolved device_id substring '%s' to %s", _device_id.c_str(),
                        full_path.c_str());
            if(full_path.find(_device_id) == std::string::npos)
                continue;
            _serial_number = get_serial_from_udev(_device_id);
            _device_id     = dev_name;

            RCLCPP_INFO(get_logger(), "Resolved device_id substring '%s' to %s", _device_id.c_str(),
                        full_path.c_str());
            RCLCPP_INFO(get_logger(), "Resolved device_id substring '%s' to %s", _device_id.c_str(),
                        full_path.c_str());
            return full_path;
        }
    }

    return "";
}

void V4L2CameraNode::load_camera_info(const std::string& device_id)
{
    if(device_id.empty())
        return;
    std::string calib_file = _calib_path;
    if(!fs::exists(calib_file))
    {
        RCLCPP_WARN(this->get_logger(), "Calibration file not found: %s", calib_file.c_str());
        return;
    }
    try
    {
        YAML::Node calib              = YAML::LoadFile(calib_file);
        _camera_info.width            = calib["image_width"].as<int>();
        _camera_info.height           = calib["image_height"].as<int>();
        _camera_info.distortion_model = calib["distortion_model"].as<std::string>();
        _camera_info.d = calib["distortion_coefficients"]["data"].as<std::vector<double>>();
        _camera_info.k = calib["camera_matrix"]["data"].as<std::array<double, 9>>();
        _camera_info.r = calib["rectification_matrix"]["data"].as<std::array<double, 9>>();
        _camera_info.p = calib["projection_matrix"]["data"].as<std::array<double, 12>>();
        RCLCPP_INFO(this->get_logger(), "Loaded calibration for device_id %s", device_id.c_str());
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load camera info: %s", e.what());
    }
}

void V4L2CameraNode::open_device()
{
    _fd = open(_device_path.c_str(), O_RDWR | O_NONBLOCK);
    if(_fd < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s", _device_path.c_str());
        return;
    }

    struct v4l2_format fmt  = {};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = _width;
    fmt.fmt.pix.height      = _height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;

    // Try MJPEG first
    if(ioctl(_fd, VIDIOC_S_FMT, &fmt) < 0)
    {
        RCLCPP_WARN(this->get_logger(), "MJPEG not supported, falling back to YUYV.");
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        if(ioctl(_fd, VIDIOC_S_FMT, &fmt) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set format (MJPEG and YUYV both failed).");
            close(_fd);
            _fd = -1;
            return;
        }
        _using_mjpeg = false;
    }
    else
    {
        _using_mjpeg = true;
    }

    struct v4l2_requestbuffers req = {};
    req.count                      = 4;
    req.type                       = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory                     = V4L2_MEMORY_MMAP;
    if(ioctl(_fd, VIDIOC_REQBUFS, &req) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to request buffers.");
        close(_fd);
        _fd = -1;
        return;
    }

    _buffers.resize(req.count);
    for(size_t i = 0; i < req.count; ++i)
    {
        struct v4l2_buffer buf = {};
        buf.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory             = V4L2_MEMORY_MMAP;
        buf.index              = i;
        if(ioctl(_fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to query buffer %zu", i);
            continue;
        }

        _buffers[i].length = buf.length;
        _buffers[i].start =
            mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, buf.m.offset);
        if(_buffers[i].start == MAP_FAILED)
        {
            RCLCPP_ERROR(this->get_logger(), "mmap failed for buffer %zu", i);
            _buffers[i].start = nullptr;
        }
    }

    for(size_t i = 0; i < req.count; ++i)
    {
        struct v4l2_buffer buf = {};
        buf.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory             = V4L2_MEMORY_MMAP;
        buf.index              = i;
        ioctl(_fd, VIDIOC_QBUF, &buf);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(_fd, VIDIOC_STREAMON, &type);
    RCLCPP_INFO(get_logger(), "Camera stream started on %s (%s)", _device_path.c_str(),
                _using_mjpeg ? "MJPEG" : "YUYV");
}

void V4L2CameraNode::capture_loop()
{
    if(_fd < 0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Camera not open, retrying...");
        open_device();
        return;
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(_fd, &fds);
    struct timeval tv = {0};
    tv.tv_sec         = 0;
    tv.tv_usec        = 500000;

    int r = select(_fd + 1, &fds, NULL, NULL, &tv);
    if(r <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "No camera data. Retrying...");
        return;
    }

    struct v4l2_buffer buf = {};
    buf.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory             = V4L2_MEMORY_MMAP;
    if(ioctl(_fd, VIDIOC_DQBUF, &buf) < 0)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to dequeue buffer.");

        struct stat st;
        if(stat(_device_path.c_str(), &st) != 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Device path no longer exists: %s. Attempting to reconnect...",
                         _device_path.c_str());
            stop_capture();

            // Optional: Re-resolve in case the device path changed (requires _device_id to be
            // stored)
            if(!_device_id.empty())
            {
                _device_path = resolve_device_id();
                RCLCPP_INFO(this->get_logger(), "Re-resolved device path: %s",
                            _device_path.c_str());
            }

            open_device();
        }

        return;
    }

    bool buffer_ok = true;

    if(buf.index >= _buffers.size())
    {
        RCLCPP_WARN(this->get_logger(), "Buffer index out of range.");
        buffer_ok = false;
    }
    else if(buf.bytesused == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Buffer is empty.");
        buffer_ok = false;
    }
    else if(buf.bytesused != _buffers[buf.index].length)
    {
        RCLCPP_DEBUG(this->get_logger(), "Buffer bytesused: %u, Expected length: %zu",
                     buf.bytesused, _buffers[buf.index].length);
        RCLCPP_DEBUG(this->get_logger(), "Buffer size mismatch.");
    }
    else if(buf.flags & V4L2_BUF_FLAG_ERROR)
    {
        RCLCPP_WARN(this->get_logger(), "Buffer error.");
        buffer_ok = false;
    }

    if(buffer_ok)
    {
        cv::Mat bgr;
        if(_using_mjpeg)
        {
            cv::Mat jpeg_data(1, buf.bytesused, CV_8UC1, _buffers[buf.index].start);
            bgr = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
            if(bgr.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Failed to decode MJPEG frame.");
                buffer_ok = false;
            }
        }
        else
        {
            cv::Mat yuyv(_height, _width, CV_8UC2, _buffers[buf.index].start);
            cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
        }

        auto stamp        = this->get_clock()->now();
        auto msg          = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();
        msg->header.stamp = stamp;
        _publisher->publish(*msg);
        {
            std::lock_guard<std::mutex> lock(_frame_mutex);
            _latest_frame = bgr.clone();
        }
        _camera_info.header.stamp = stamp;
        _camera_info_pub->publish(_camera_info);
    }

    if(ioctl(_fd, VIDIOC_QBUF, &buf) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to requeue buffer: %s", strerror(errno));
    }
}

void V4L2CameraNode::stop_capture()
{
    if(_fd >= 0)
    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(_fd, VIDIOC_STREAMOFF, &type);
        for(auto& b : _buffers)
        {
            munmap(b.start, b.length);
        }
        close(_fd);
        _fd = -1;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<V4L2CameraNode>();
    node->_calibration_server->start(); // safe, node is fully shared now
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}