# NOTE: This is totally vibe coded btw ♡
import argparse
import sys
import time
import cv2


def make_aruco_dict(dict_name=cv2.aruco.DICT_6X6_250):
    """Return an ArUco dictionary object, compatible across OpenCV versions."""
    try:
        # Preferred newer API
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_name)
    except Exception:
        # Older API fallback
        aruco_dict = cv2.aruco.Dictionary_get(dict_name)
    return aruco_dict


def make_detector_parameters():
    try:
        params = cv2.aruco.DetectorParameters_create()
    except Exception:
        # Older API fallback
        params = cv2.aruco.DetectorParameters()
    return params


def detect_and_draw(frame, aruco_dict, parameters):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect markers using a compatibility approach:
    # - Newer OpenCV versions provide cv2.aruco.ArucoDetector with .detect()
    # - Older versions use cv2.aruco.detectMarkers()
    corners = None
    ids = None
    rejected = None
    try:
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            # detector.detect returns corners, ids, rejected in newer builds
            corners, ids, rejected = detector.detect(gray)
        else:
            # fallback to older API which takes parameters kwarg
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, aruco_dict, parameters=parameters)
    except Exception:
        # Final fallback: try detectMarkers without parameters (some builds)
        try:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
        except Exception:
            corners, ids, rejected = None, None, None

    if ids is not None:
        # draw detected markers (outline) using OpenCV helper if available
        try:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        except Exception:
            # if helper not available, we'll draw per-marker below
            pass

        # additionally draw a polygon outline + axis-aligned bounding box per marker
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            pts = marker_corners.reshape((4, 2)).astype(int)

            # polygon outline (in case drawDetectedMarkers wasn't available)
            cv2.polylines(frame, [pts], isClosed=True,
                          color=(0, 255, 0), thickness=2)

            # axis-aligned bounding box around the marker
            x_min = int(pts[:, 0].min())
            y_min = int(pts[:, 1].min())
            x_max = int(pts[:, 0].max())
            y_max = int(pts[:, 1].max())
            cv2.rectangle(frame, (x_min, y_min),
                          (x_max, y_max), (255, 0, 0), 2)

            # label: draw filled background then text for readability
            label = str(int(marker_id))
            (tw, th), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            # place label at top-left corner of bounding box
            lx, ly = x_min, max(y_min - 6, th)
            cv2.rectangle(frame, (lx - 2, ly - th - 2), (lx +
                          tw + 2, ly + baseline + 2), (255, 0, 0), cv2.FILLED)
            cv2.putText(frame, label, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return frame, gray, ids


def open_camera(device_index, width=None, height=None):
    # On Windows, using CAP_DSHOW often reduces delay when opening webcams
    cap = cv2.VideoCapture(device_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        # try without CAP_DSHOW as a fallback
        cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        return None
    if width is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height is not None:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def main():
    parser = argparse.ArgumentParser(
        description='Live ArUco marker detector (OpenCV).')
    parser.add_argument('--device', '-d', type=int, default=0,
                        help='Camera device index or video file path (default: 0)')
    parser.add_argument('--width', type=int, default=None,
                        help='Optional capture width')
    parser.add_argument('--height', type=int, default=None,
                        help='Optional capture height')
    parser.add_argument('--no-display', action='store_true',
                        help="Don't show GUI windows")
    args = parser.parse_args()

    cap = open_camera(args.device, args.width, args.height)
    if cap is None:
        print(f"ERROR: Unable to open camera device {args.device}")
        sys.exit(1)

    aruco_dict = make_aruco_dict()
    parameters = make_detector_parameters()

    print('Starting camera. Press q to quit.')

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print('Warning: empty frame received, retrying...')
                time.sleep(0.1)
                continue

            out_frame, gray, ids = detect_and_draw(
                frame, aruco_dict, parameters)

            # Log ids to console (None if nothing detected)
            if ids is not None:
                print('Detected ids:', ids.flatten())

            if not args.no_display:
                cv2.imshow('ArUco - color', out_frame)
                # cv2.imshow('ArUco - gray', gray)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
            else:
                # if no display, still allow safe exit with Ctrl-C
                time.sleep(0.01)

    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        cap.release()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
