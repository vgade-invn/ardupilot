/*
    This class has been implemented based on
    yavta -- Yet Another V4L2 Test Application written by:
    Laurent Pinchart <laurent.pinchart@ideasonboard.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP ||\
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MR100
#include "VideoIn.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

bool VideoIn::get_frame(Frame &frame)
{
    if (!_streaming) {
        if (!_set_streaming(true)) {
            AP_HAL::panic("couldn't start streaming\n");
            return false;
        }
        _streaming = true;
    }

    return _dequeue_frame(frame);
}

void VideoIn::put_frame(Frame &frame)
{
    _queue_buffer((uint32_t)frame.buf_index);
}

bool VideoIn::open_device(const char *device_path, uint32_t memtype)
{
    struct v4l2_capability cap;
    int ret;

    _fd = -1;
    _buffers = nullptr;
    _fd = open(device_path, O_RDWR|O_CLOEXEC);
    _memtype = memtype;
    if (_fd < 0) {
        hal.console->printf("Error opening device %s: %s (%d).\n",
                            device_path,
                            strerror(errno), errno);
        return false;
    }
    printf("Opened %s\n", device_path);

    memset(&cap, 0, sizeof cap);
    ret = ioctl(_fd, VIDIOC_QUERYCAP, &cap);
    if (ret < 0) {
        hal.console->printf("Error querying caps\n");
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        hal.console->printf("Error opening device %s: is not a video capture device\n",
                            device_path);
        close(_fd);
        _fd = -1;
        return false;
    }

    return true;
}

bool VideoIn::allocate_buffers(uint32_t nbufs)
{
    struct v4l2_requestbuffers rb;
    struct v4l2_buffer buf;
    struct buffer *buffers;
    unsigned int i;
    int ret;

    memset(&rb, 0, sizeof rb);
    rb.count = nbufs;
    rb.type = (v4l2_buf_type) V4L2_CAP_VIDEO_CAPTURE;
    rb.memory = (v4l2_memory) _memtype;

    ret = ioctl(_fd, VIDIOC_REQBUFS, &rb);
    if (ret < 0) {
        printf("Unable to request buffers: %s (%d).\n", strerror(errno), errno);
        return ret;
    }

    buffers = (struct buffer *)malloc(rb.count * sizeof buffers[0]);
    if (buffers == nullptr) {
        hal.console->printf("Unable to allocate buffers\n");
        return false;
    }

    for (i=0; i < rb.count; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.index = i;
        buf.type = rb.type;
        buf.memory = rb.memory;
        ret = ioctl(_fd, VIDIOC_QUERYBUF, &buf);
        if (ret < 0) {
            hal.console->printf("Unable to query buffer %u: %s(%d).\n", i,
                                strerror(errno), errno);
            return false;
        }

        switch (_memtype) {
        case V4L2_MEMORY_MMAP:
            buffers[i].mem = mmap(0, buf.length, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, _fd, buf.m.offset);
            if (buffers[i].mem == MAP_FAILED) {
                hal.console->printf("Unable to map buffer %u: %s (%d)\n", i,
                                    strerror(errno), errno);
                return false;
            }
            buffers[i].size = buf.length;
            break;
        case V4L2_MEMORY_USERPTR:
            ret = posix_memalign(&buffers[i].mem, getpagesize(), buf.length);
            if (ret < 0) {
                hal.console->printf("Unable to allocate buffer %u (%d)\n", i,
                                    ret);
                return false;
            }
            buffers[i].size = buf.length;
            break;
        default:
            return false;
        }
    }
    _nbufs = rb.count;
    _buffers = buffers;
    return true;
}

void VideoIn::get_pixel_formats(std::vector<uint32_t> *formats)
{
    struct v4l2_fmtdesc fmtdesc;

    memset(&fmtdesc, 0, sizeof fmtdesc);

    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    while (ioctl(_fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        formats->insert(formats->begin(), fmtdesc.pixelformat);
        fmtdesc.index++;
    }
}

bool VideoIn::set_format(uint32_t *width, uint32_t *height, uint32_t *format,
                         uint32_t *bytesperline, uint32_t *sizeimage)
{
    struct v4l2_format fmt;
    struct v4l2_input input;
    int ret;

    // set type camera. Only needed on some cameras, and harmless on others
    memset(&input, 0, sizeof input);
    input.index = 0;
    input.type = V4L2_INPUT_TYPE_CAMERA;
    ioctl(_fd, VIDIOC_S_INPUT, &input);

    printf("format requested (%08x) %ux%u\n",
           *format, *width, *height);

    memset(&fmt, 0, sizeof fmt);
    fmt.type = (v4l2_buf_type) V4L2_CAP_VIDEO_CAPTURE;
    fmt.fmt.pix.width = *width;
    fmt.fmt.pix.height = *height;
    fmt.fmt.pix.pixelformat = *format;
    fmt.fmt.pix.colorspace = V4L2_COLORSPACE_REC709;

    ret = ioctl(_fd, VIDIOC_S_FMT, &fmt);
    if (ret < 0) {
        hal.console->printf("VideoIn: unable to set format: %s (%d).\n",
                            strerror(errno), errno);
        return false;
    }

    /* warn if format different from the one that was supposed
     * to be set
     */
    if ((fmt.fmt.pix.pixelformat != *format) ||
        (fmt.fmt.pix.width != *width) ||
        (fmt.fmt.pix.height != *height)) {
        hal.console->printf("format set to (%08x)"
                            "%ux%u buffer size %u field : %d\n",
                            fmt.fmt.pix.pixelformat, fmt.fmt.pix.width,
                            fmt.fmt.pix.height, fmt.fmt.pix.sizeimage,
                            fmt.fmt.pix.field);
    }

    *width = fmt.fmt.pix.width;
    *height = fmt.fmt.pix.height;
    *format = fmt.fmt.pix.pixelformat;
    *bytesperline = fmt.fmt.pix.bytesperline;
    *sizeimage = fmt.fmt.pix.sizeimage;

    if (*sizeimage == 0 && *format == V4L2_PIX_FMT_YUV420) {
        // the MR100 camera doesn't return a size, so calculate it here
        *sizeimage = (*width) * (*height) * 6 / 4;
        *bytesperline = (*width) * 6 / 4;
    }

    printf("format set to (%08x) %ux%u size=%u\n",
           *format, *width, *height, *sizeimage);
    
    return true;
}

bool VideoIn::set_crop(uint32_t left, uint32_t top,
                       uint32_t width, uint32_t height)
{
    struct v4l2_crop crop;
    int ret;

    memset(&crop, 0, sizeof crop);
    crop.c.top = top;
    crop.c.left = left;
    crop.c.width = width;
    crop.c.height = height;
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ret = ioctl(_fd, VIDIOC_S_CROP, &crop);

    if (ret < 0) {
        hal.console->printf("VideoIn: unable to set crop: %s (%d).\n",
                            strerror(errno), errno);
        return false;
    }

    return true;
}

void VideoIn::prepare_capture()
{
    unsigned int i;

    /* Queue the buffers. */
    for (i = 0; i < _nbufs; ++i) {
        _queue_buffer(i);
    }
}

void VideoIn::shrink_8bpp(uint8_t *buffer, uint8_t *new_buffer,
                          uint32_t width, uint32_t height, uint32_t left,
                          uint32_t selection_width, uint32_t top,
                          uint32_t selection_height, uint32_t fx, uint32_t fy)
{
    const uint32_t out_width = selection_width / fx;
    const uint32_t out_height = selection_height / fy;
    const uint32_t fx_fy = fx * fy;

#if 0
    printf("shrink %ux%u -> %ux%u %ux%u %ux%u\n",
           width, height, left, top, selection_width, selection_height,
           out_width, out_height);
#endif
    
    for (uint16_t y = 0; y < out_height; y++) {
        uint8_t *row = &new_buffer[y*out_width];
        uint32_t block_position = left + width * (top+y*fy);
        for (uint16_t x = 0; x < out_width; x++) {
            uint32_t px = 0;

            uint32_t width_sum = 0;
            for (uint16_t k = 0; k < fy; k++) {
                for (uint16_t kk = 0; kk < fx; kk++) {
                    const uint32_t ofs = block_position + kk + width_sum;
                    px += buffer[ofs];
                }
                width_sum += width;
            }

            row[x] = px / (fx_fy);

            block_position += fx;
        }
    }
}

void VideoIn::crop_8bpp(uint8_t *buffer, uint8_t *new_buffer,
                        uint32_t width, uint32_t left, uint32_t crop_width,
                        uint32_t top, uint32_t crop_height)
{
    uint32_t crop_x = left + crop_width;
    uint32_t crop_y = top + crop_height;
    uint32_t buffer_index = top * width;
    uint32_t new_buffer_index = 0;

    printf("crop %ux? -> %ux%u %ux%u\n", width, left, top, crop_width, crop_height);
    
    for (uint32_t j = top; j < crop_y; j++) {
        for (uint32_t i = left; i < crop_x; i++) {
            new_buffer[i - left + new_buffer_index] =  buffer[i + buffer_index];
        }
        buffer_index += width;
        new_buffer_index += crop_width;
    }
}

void VideoIn::yuyv_to_grey(uint8_t *buffer, uint32_t buffer_size,
                           uint8_t *new_buffer)
{
    uint32_t new_buffer_position = 0;

    for (uint32_t i = 0; i < buffer_size; i += 2) {
        new_buffer[new_buffer_position] = buffer[i];
        new_buffer_position++;
    }
}

void VideoIn::yuv420_to_grey(uint8_t *buffer, uint32_t buffer_size,
                             uint8_t *new_buffer)
{
    // the Y values are in the first part of the buffer
    memcpy(new_buffer, buffer, buffer_size*4/6);
}

uint32_t VideoIn::_timeval_to_us(struct timeval& tv)
{
    return (1.0e6 * tv.tv_sec + tv.tv_usec);
}

void VideoIn::_queue_buffer(int index)
{
    int ret;
    struct v4l2_buffer buf;

    memset(&buf, 0, sizeof buf);
    buf.index = index;
    buf.type = (v4l2_buf_type) V4L2_CAP_VIDEO_CAPTURE;
    buf.memory = (v4l2_memory) _memtype;
    buf.length = _buffers[index].size;
    if (_memtype == V4L2_MEMORY_USERPTR) {
        buf.m.userptr = (unsigned long) _buffers[index].mem;
    }

    ret = ioctl(_fd, VIDIOC_QBUF, &buf);
    if (ret < 0) {
        hal.console->printf("Unable to queue buffer : %s (%d).\n",
                            strerror(errno), errno);
    }
}

bool VideoIn::_set_streaming(bool enable)
{
    int type = V4L2_CAP_VIDEO_CAPTURE;
    int ret;

    ret = ioctl(_fd, enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type);
    if (ret < 0) {
        printf("Unable to %s streaming: %s (%d).\n",
               enable ? "start" : "stop", strerror(errno), errno);
        return false;
    }

    return true;
}

bool VideoIn::_dequeue_frame(Frame &frame)
{
    uint32_t t0, t1;
    static uint32_t t2;

    t0 = AP_HAL::micros();
    uint32_t dt0 = t0 - t2;
    
    struct v4l2_buffer buf;
    int ret;

    /* Dequeue a buffer. */
    memset(&buf, 0, sizeof buf);
    buf.type = (v4l2_buf_type) V4L2_CAP_VIDEO_CAPTURE;
    buf.memory = (v4l2_memory) _memtype;

    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(_fd, &fds);
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    ret = select(_fd+1, &fds, NULL, NULL, &tv);
    if (ret != 1) {
        hal.console->printf("_dequeue_frame select error\n");
        return false;
    }

    uint64_t select_timestamp = AP_HAL::micros64();
    
    ret = ioctl(_fd, VIDIOC_DQBUF, &buf);
    if (ret < 0) {
        if (errno != EIO) {
            hal.console->printf("Unable to dequeue buffer: %s (%d).\n",
                                strerror(errno), errno);
            return false;
        }
        buf.type = (v4l2_buf_type) V4L2_CAP_VIDEO_CAPTURE;
        buf.memory = (v4l2_memory) _memtype;
        if (_memtype == V4L2_MEMORY_USERPTR) {
            buf.m.userptr = (unsigned long)_buffers[buf.index].mem;
        }
    }

    frame.data = _buffers[buf.index].mem;
    frame.buf_index = buf.index;
    frame.timestamp = select_timestamp;
    frame.sequence = buf.sequence;

    t1 = AP_HAL::micros();
    uint32_t dt1 = t1 - t0;
    if (false) printf("dt0=%u dt1=%u timestamp=%llu seq=%u ret=%d\n", dt0, dt1, frame.timestamp, frame.sequence, ret);

    t2 = t1;
    
    return true;
}

#endif
