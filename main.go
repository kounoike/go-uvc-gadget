package main

/*
#include <errno.h>
#include <linux/usb/ch9.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>
#include "uvc.h"
#define PU_BRIGHTNESS_MIN_VAL 0
#define PU_BRIGHTNESS_MAX_VAL 255
#define PU_BRIGHTNESS_STEP_SIZE 1
#define PU_BRIGHTNESS_DEFAULT_VAL 127
struct my_uvc_streaming_control {
	__u16 bmHint;
	__u8  bFormatIndex;
	__u8  bFrameIndex;
	__u32 dwFrameInterval;
	__u16 wKeyFrameRate;
	__u16 wPFrameRate;
	__u16 wCompQuality;
	__u16 wCompWindowSize;
	__u16 wDelay;
	__u8 dwMaxVideoFrameSize[4];
	__u8 dwMaxPayloadTransferSize[4];
	__u8 dwClockFrequency[4];
	__u8  bmFramingInfo;
	__u8  bPreferedVersion;
	__u8  bMinVersion;
	__u8  bMaxVersion;
} __attribute__((__packed__));
*/
import "C"

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"syscall"
	"time"
	"unsafe"

	"github.com/paypal/gatt/linux/gioctl"
)

const (
	typeV4L2 = 86 // 'V'
	typeUvc  = 85 // 'U'
)

type uvcStreamingControl struct {
	bmHint                   uint16
	bFormatIndex             uint8
	bFrameIndex              uint8
	dwFrameInterval          uint32
	wKeyFrameRate            uint16
	wPFrameRate              uint16
	wCompQuality             uint16
	wCompWindowSize          uint16
	wDelay                   uint16
	dwMaxVideoFrameSize      uint32
	dwMaxPayloadTransferSize uint32
	dwClockFrequency         uint32
	bmFramingInfo            uint8
	bPreferedVersion         uint8
	bMinVersion              uint8
	bMaxVersion              uint8
}

type packedUvcStreamingControl struct {
	packed [34]byte
}

type deviceInfo struct {
	uvcFd            int
	isStreaming      bool
	probe            C.struct_uvc_streaming_control
	commit           C.struct_uvc_streaming_control
	requestErrorCode C.struct_uvc_request_data
	brightnessValue  uint8
	control          uint8
}

var (
	// #define	VIDIOC_SUBSCRIBE_EVENT	 _IOW('V', 90, struct v4l2_event_subscription)
	vidiocSubscribeEvent = gioctl.IoW(typeV4L2, 90, uintptr(C.sizeof_struct_v4l2_event_subscription))
	// #define	VIDIOC_DQEVENT		 _IOR('V', 89, struct v4l2_event)
	vidiocDqEvent = gioctl.IoR(typeV4L2, 89, uintptr(C.sizeof_struct_v4l2_event))
	// #define UVCIOC_SEND_RESPONSE		_IOW('U', 1, struct uvc_request_data)
	uvciocSendResponse = gioctl.IoW(typeUvc, 1, uintptr(C.sizeof_struct_uvc_request_data))

	dev deviceInfo
)

func dwToBytes(dw uint32, bytes *[4]C.uchar) {
	b := make([]byte, 4)
	binary.LittleEndian.PutUint32(b[0:], dw)
	bytes[0] = C.uchar(b[0])
	bytes[1] = C.uchar(b[1])
	bytes[2] = C.uchar(b[2])
	bytes[3] = C.uchar(b[3])
}

func uvcFillStreamingControl(argCtrl *C.struct_uvc_streaming_control, iFrame int, iFormat int) {
	var ctrl C.struct_my_uvc_streaming_control

	ctrl.bmHint = 1
	ctrl.bFormatIndex = C.uchar(iFormat + 1)
	ctrl.bFrameIndex = C.uchar(iFrame + 1)
	ctrl.dwFrameInterval = 666666

	dwToBytes(1280*720*2, &ctrl.dwMaxVideoFrameSize)
	dwToBytes(1280*720*2, &ctrl.dwMaxPayloadTransferSize)
	ctrl.bmFramingInfo = 3
	ctrl.bPreferedVersion = 1
	ctrl.bMaxVersion = 1
	*argCtrl = *(*C.struct_uvc_streaming_control)(unsafe.Pointer(&ctrl))
	log.Print(argCtrl)

	// var packedCtrl packedUvcStreamingControl
	// var buf = bytes.NewBuffer(make([]byte, 0, len(packedCtrl.packed)))
	// if err := binary.Write(buf, binary.LittleEndian, &tmpCtrl); err != nil {
	// 	log.Fatal("Write buf error:", err)
	// }
	// if err := binary.Read(buf, binary.LittleEndian, &packedCtrl); err != nil {
	// 	log.Fatal("Read buf error:", err)
	// }
	// *ctrl = *(*C.struct_uvc_streaming_control)(unsafe.Pointer(&packedCtrl))
}

func uvcInit(fd int) {
	var subscription C.struct_v4l2_event_subscription
	subscription._type = C.UVC_EVENT_CONNECT
	gioctl.Ioctl(uintptr(fd), vidiocSubscribeEvent, uintptr(unsafe.Pointer(&subscription)))
	subscription._type = C.UVC_EVENT_DISCONNECT
	gioctl.Ioctl(uintptr(fd), vidiocSubscribeEvent, uintptr(unsafe.Pointer(&subscription)))
	subscription._type = C.UVC_EVENT_SETUP
	gioctl.Ioctl(uintptr(fd), vidiocSubscribeEvent, uintptr(unsafe.Pointer(&subscription)))
	subscription._type = C.UVC_EVENT_DATA
	gioctl.Ioctl(uintptr(fd), vidiocSubscribeEvent, uintptr(unsafe.Pointer(&subscription)))
	subscription._type = C.UVC_EVENT_STREAMON
	gioctl.Ioctl(uintptr(fd), vidiocSubscribeEvent, uintptr(unsafe.Pointer(&subscription)))
	subscription._type = C.UVC_EVENT_STREAMOFF
	gioctl.Ioctl(uintptr(fd), vidiocSubscribeEvent, uintptr(unsafe.Pointer(&subscription)))

	uvcFillStreamingControl(&dev.probe, 0, 0)
	uvcFillStreamingControl(&dev.commit, 0, 0)
}

func uvcHandleControl(req uint8, cs uint8, entityId uint8, len uint8, resp *C.struct_uvc_request_data) {
	log.Print("req: ", req, " cs: ", cs, " entityId: ", entityId, " len: ", len, " resp: ", resp)
	switch entityId {
	case 0:
		log.Fatal(fmt.Sprintf("entity_id 0 not supported req:%d cs:%d", req, cs))
	case 1:
		log.Print("entity_id 1")
		switch cs {
		case C.UVC_CT_AE_MODE_CONTROL:
			switch req {
			case C.UVC_SET_CUR:
				resp.data[0] = 0x01
				resp.length = 1
			case C.UVC_GET_INFO:
				resp.data[0] = 0x03
				resp.length = 1
			case C.UVC_GET_CUR, C.UVC_GET_DEF, C.UVC_GET_RES:
				resp.data[0] = 0x02
				resp.length = 1
			default:
				resp.length = -C.EL2HLT
			}
		}
	case 2:
		switch cs {
		case C.UVC_PU_BRIGHTNESS_CONTROL:
			switch req {
			case C.UVC_SET_CUR:
				resp.data[0] = 0x00
				resp.length = C.int(len)
			case C.UVC_GET_MIN:
				resp.data[0] = C.PU_BRIGHTNESS_MIN_VAL
				resp.length = 2
			case C.UVC_GET_MAX:
				resp.data[0] = C.PU_BRIGHTNESS_MAX_VAL
				resp.length = 2
			case C.UVC_GET_CUR:
				resp.data[0] = C.PU_BRIGHTNESS_DEFAULT_VAL
				resp.length = 1
			case C.UVC_GET_INFO:
				resp.data[0] = 0x03
				resp.length = 1
			case C.UVC_GET_DEF:
				resp.data[0] = C.PU_BRIGHTNESS_DEFAULT_VAL
				resp.length = 2
			case C.UVC_GET_RES:
				resp.data[0] = C.PU_BRIGHTNESS_STEP_SIZE
				resp.length = 2
			}
		}
	}
}

func uvcHandleStreaming(req uint8, cs uint8, resp *C.struct_uvc_request_data) {
	log.Print(fmt.Sprintf("uvcHandleStreaming req:%d cs:%d", req, cs))
	if cs == C.UVC_VS_PROBE_CONTROL || cs == C.UVC_VS_COMMIT_CONTROL {
		switch req {
		case C.UVC_SET_CUR:
			log.Print("UVC_SET_CUR")
			dev.control = cs
			resp.length = 34
		case C.UVC_GET_CUR:
			log.Print("UVC_GET_CUR")
			if cs == C.UVC_VS_PROBE_CONTROL {
				resp.data = *(*[60]C.uchar)(unsafe.Pointer(&dev.probe))
				log.Print("probe:", dev.probe)
			}
			if cs == C.UVC_VS_COMMIT_CONTROL {
				resp.data = *(*[60]C.uchar)(unsafe.Pointer(&dev.commit))
				log.Print("commit:", dev.commit)
			}
			resp.length = C.sizeof_struct_uvc_streaming_control
			log.Print("resp:", resp)
		case C.UVC_GET_MIN, C.UVC_GET_DEF:
			log.Print("UVC_GET_MIN/DEF")
			var ctrl C.struct_uvc_streaming_control
			uvcFillStreamingControl(&ctrl, 0, 0)
			resp.data = *(*[60]C.uchar)(unsafe.Pointer(&ctrl))
			resp.length = C.sizeof_struct_uvc_streaming_control
		case C.UVC_GET_MAX:
			log.Print("UVC_GET_MAX")
			var ctrl C.struct_uvc_streaming_control
			uvcFillStreamingControl(&ctrl, 0, 0) // should be -1(MAX)
			buf := new(bytes.Buffer)
			err := binary.Write(buf, binary.LittleEndian, ctrl)
			if err != nil {
				log.Fatal("Can't write to buffer:", err)
			}
			for i := 60; i < 60; i++ {
				resp.data[i] = C.uchar(buf.Bytes()[i])
			}
			resp.length = C.sizeof_struct_uvc_streaming_control
		case C.UVC_GET_RES:
			log.Print("UVC_GET_RES")
			// TODO: What to do???
		case C.UVC_GET_LEN:
			log.Print("UVC_GET_LEN")
			resp.data[0] = 0x00
			resp.data[1] = 0x22
			resp.length = 2
		case C.UVC_GET_INFO:
			log.Print("UVC_GET_INFO")
			resp.data[0] = 0x03
			resp.length = 1
		default:
			log.Fatal("Unknown requst")
		}
	}
	log.Print(fmt.Sprintf("uvcHandleStreaming req:%d cs:%d Done.", req, cs))
}

func uvcHandleData(data [64]byte) {
	log.Print("uvcHandleData", data)
	var target *C.struct_uvc_streaming_control
	switch dev.control {
	case C.UVC_VS_PROBE_CONTROL:
		log.Print("UVC_VS_PROBE_CONTROL")
		target = &dev.probe
	case C.UVC_VS_COMMIT_CONTROL:
		log.Print("UVC_VS_COMMIT_CONTROL")
		target = &dev.commit
	default:
		log.Fatal("Unknown Control")
	}
	log.Print("target", target)
}

func uvcHandleEvent(fd int) {
	var v4l2Event C.struct_v4l2_event
	var resp C.struct_uvc_request_data

	gioctl.Ioctl(uintptr(fd), vidiocDqEvent, uintptr(unsafe.Pointer(&v4l2Event)))
	log.Print(v4l2Event)
	log.Print("Event Type:", fmt.Sprintf("%x", v4l2Event._type))
	switch v4l2Event._type {
	case C.UVC_EVENT_CONNECT:
		log.Print("UVC_EVENT_CONNECT")
		return
	case C.UVC_EVENT_DISCONNECT:
		log.Print("UVC_EVENT_DISCONNECT")
		return
	case C.UVC_EVENT_SETUP:
		log.Print("UVC_EVENT_SETUP")
		uvcEvent := (*C.struct_uvc_event)(unsafe.Pointer(&v4l2Event.u))
		log.Print("uvcEventp: ", uvcEvent)
		req := (*C.struct_usb_ctrlrequest)(unsafe.Pointer(uvcEvent))
		log.Print("req: ", req)
		if req.bRequestType&C.USB_TYPE_MASK == C.USB_TYPE_STANDARD {
			log.Print("USB_TYPE_STANDARD")
		}
		if req.bRequestType&C.USB_TYPE_MASK == C.USB_TYPE_CLASS {
			log.Print("USB_TYPE_CLASS")
			if req.bRequestType&C.USB_RECIP_MASK == C.USB_RECIP_INTERFACE {
				switch req.wIndex & 0xff {
				case C.UVC_INTF_CONTROL:
					uvcHandleControl(uint8(req.bRequest), uint8(req.wValue>>8), uint8(req.wIndex>>8), uint8(req.wLength), &resp)
				case C.UVC_INTF_STREAMING:
					uvcHandleStreaming(uint8(req.bRequest), uint8(req.wValue>>8), &resp)
				}
			}
		}
	case C.UVC_EVENT_DATA:
		log.Print("UVC_EVENT_DATA")
		uvcHandleData(v4l2Event.u)
	case C.UVC_EVENT_STREAMON:
		log.Fatal("UVC_EVENT_STREAMON not supported now")
	case C.UVC_EVENT_STREAMOFF:
		log.Fatal("UVC_EVENT_STREAMOFF not supported now")
	default:
		log.Print(fmt.Sprintf("Not supported type: %x", v4l2Event._type))
	}
	gioctl.Ioctl(uintptr(fd), uvciocSendResponse, uintptr(unsafe.Pointer(&resp)))
}

func main() {
	startTime := time.Now()
	fd, err := syscall.Open("/dev/video0", syscall.O_RDWR|syscall.O_NONBLOCK, 0666)
	if err != nil {
		log.Fatal("Open error: ", err)
	}
	log.Print("Hello ", fd)
	uvcInit(fd)

	// epfd, err := syscall.EpollCreate(1)
	// if err != nil {
	// 	log.Fatal("EpollCreate error:", err)
	// }
	// event := &syscall.EpollEvent{
	// 	Events: syscall.EPOLLIN | syscall.EPOLLOUT | syscall.EPOLLERR,
	// 	Fd:     int32(fd),
	// }
	// err = syscall.EpollCtl(epfd, syscall.EPOLL_CTL_ADD, int(fd), event)
	// if err != nil {
	// 	log.Fatal("EpollCtl error:", err)
	// }

	var cnt = 0
	for {
		duration := time.Since(startTime)
		if duration.Seconds() > 30.0 {
			break
		}
		cnt++
		if cnt > 1000 {
			log.Print("1000 loop after no message")
			cnt = 0
		}

		// var events [1]syscall.EpollEvent
		// n, err := syscall.EpollWait(epfd, events[:], 100)
		// log.Print(n, err, events, events[0].Events)
		// if n > 0 && events[0].Events == syscall.EPOLLERR {
		// 	var v4l2Event C.struct_v4l2_event
		// 	gioctl.Ioctl(uintptr(fd), vidiocDqEvent, uintptr(unsafe.Pointer(&v4l2Event)))
		// 	log.Print(v4l2Event)
		// }

		// var r syscall.FdSet
		// r.Bits[fd/64] = 1 << (fd % 64)
		// var w syscall.FdSet
		// w.Bits[fd/64] = 1 << (fd % 64)
		var x syscall.FdSet
		x.Bits[fd/64] = 1 << (fd % 64)
		n, err := syscall.Select(fd+1, nil, nil, &x, nil)
		// log.Print(n, err, r, w, x)
		if err != nil {
			log.Fatal("Select error:", err)
		}
		if x.Bits[fd/64]&(1<<(fd%64)) != 0 {
			cnt = 0
			log.Print("***select done***", n, x)
			uvcHandleEvent(fd)
		}
	}
}
