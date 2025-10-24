from pickle import GLOBAL, NONE
import cv2
import numpy as np
import time
import urllib.request
#CAMERANUMBER = 0
CAMERANUMBER =  'http://192.168.43.1:8080/video'
#CAMERANUMBER = 'http://192.168.137.234:81/stream'
TRACKBAR1 = 146
TRACKBAR2 = 112
AREA = 2400

red_lower = np.array([160, 100, 100], np.uint8)
red_upper = np.array([180, 255, 255], np.uint8)

green_lower = np.array([40, 50, 50], np.uint8)
green_upper = np.array([80, 255, 255], np.uint8)

blue_lower = np.array([94, 80, 2], np.uint8)
blue_upper = np.array([130, 255, 255], np.uint8)

yellow_lower = np.array([20, 150, 150], np.uint8)
yellow_upper = np.array([35, 255, 255], np.uint8)

frameWidth = 640
frameHeight = 680
cap = None
CAM_URL = None
USE_SNAPSHOT = False

cropping = False

x_start, y_start, x_end, y_end = 0, 0, 0, 0

mouseX = 100
mouseY = 100
hsv_img = None
upper = np.array([35, 55, 100])
lower = np.array([0, 0, 0])
POSX = None
POSY = None

# --- ROI calibration (zone de tri) ---
# Points cliqués dans l'image affichée (600x300)
ROI_POINTS_DISPLAY = []  # [(x,y), ...] en pixels de l'affichage 600x300
ROI_READY = False

# Dimensions réelles de la zone (en cm) - à définir manuellement dans le code
ZONE_WIDTH_CM = 15.0
ZONE_HEIGHT_CM = 15.0

# Homographie pixel->cm (calculée quand 4 points sont saisis)
H_PIX_TO_CM = None

# Position relative en cm (dans la zone), calculée à partir de l'homographie
REL_POSX_CM = None
REL_POSY_CM = None


def getPos():
    return POSX, POSY

def crop(img):
    cropped_image = img[0:600, 0:300]
    return cropped_image

def mouseClickHandler(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img, (x, y), 100, (255, 0, 0), -1)
        print(f"x={x} y={y}")


def detect_color(mask, imgContour, color_name, color_bgr):
    global POSX, POSY
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cont in contours:
        area = cv2.contourArea(cont)
        if area > 300:
            moments = cv2.moments(cont)
            if moments["m00"] != 0:
                # Get center coordinates
                cx = int(moments["m10"] / moments["m00"])  # Adding 3 to fix the X offset
                cy = int(moments["m01"] / moments["m00"])
                
                # Correct X coordinate by adding 3 to compensate for offset
                POSX = cx + 3  # Adding 3 to fix the X offset
                POSY = cy
                
                # Draw visualization
                x, y, w, h = cv2.boundingRect(cont)
                cv2.rectangle(imgContour, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.putText(imgContour, f"{color_name} Colour", (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
                
                # Draw center point with corrected coordinates
                cv2.circle(imgContour, (POSX, POSY), 5, (255, 255, 255), -1)
                cv2.putText(imgContour, f"({POSX}, {POSY})", (POSX+10, POSY),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

def add_roi_point_display(x, y):
    """
    Ajoute un point ROI en coordonnées d'affichage (600x300).
    Appeler 4 fois pour définir la zone de tri.
    """
    global ROI_POINTS_DISPLAY, ROI_READY
    if ROI_READY:
        return
    if x is None or y is None:
        return
    ROI_POINTS_DISPLAY.append((int(x), int(y)))
    if len(ROI_POINTS_DISPLAY) >= 4:
        ROI_READY = True

def reset_roi():
    """Réinitialise la zone de tri."""
    global ROI_POINTS_DISPLAY, ROI_READY, H_PIX_TO_CM
    ROI_POINTS_DISPLAY = []
    ROI_READY = False
    H_PIX_TO_CM = None

def getImageCoordinates():
    global hsv_img, upper, lower, cropping, x_start, y_start, x_end, y_end, cap, USE_SNAPSHOT, CAM_URL

    try:
        # Ensure camera is initialized
        if cap is None or not cap.isOpened():
            if not init_camera():
                print("Failed to initialize camera")
                return None

        # Try to read frame with retry mechanism
        max_retries = 3
        retry_count = 0
        success = False
        img = None

        while retry_count < max_retries and not success:
            try:
                if cap is not None and cap.isOpened():
                    success, img = cap.read()
                elif USE_SNAPSHOT and isinstance(CAM_URL, str):
                    with urllib.request.urlopen(CAM_URL, timeout=3) as resp:
                        jpg = resp.read()
                    img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    success = img is not None
                else:
                    success = False

                if not success or img is None:
                    print(f"Failed to capture image (attempt {retry_count + 1}/{max_retries})")
                    retry_count += 1
                    time.sleep(0.1)
                    if retry_count == max_retries - 1:
                        if not init_camera():
                            return None
            except Exception as e:
                print(f"Error reading frame: {str(e)}")
                retry_count += 1
                time.sleep(0.1)

        if not success or img is None:
            print("Failed to capture image from camera after retries")
            return None

        # Ensure image is not None before processing
        if img is None:
            return None

        # Crop and rotate image
        try:
            img = crop(img)
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        except Exception as e:
            print(f"Error in image processing: {e}")
            return None

        # Create contour image
        imgContour = img.copy()
        
        # Convert to HSV
        try:
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        except Exception as e:
            print(f"Error in HSV conversion: {e}")
            return None

        # Create color masks
        try:
            red_mask = cv2.inRange(hsv_img, red_lower, red_upper)
            green_mask = cv2.inRange(hsv_img, green_lower, green_upper)
            blue_mask = cv2.inRange(hsv_img, blue_lower, blue_upper)
            yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)

            # Apply dilation
            kernel = np.ones((5, 5), "uint8")
            red_mask = cv2.dilate(red_mask, kernel)
            green_mask = cv2.dilate(green_mask, kernel)
            blue_mask = cv2.dilate(blue_mask, kernel)
            yellow_mask = cv2.dilate(yellow_mask, kernel)

            # Si la zone de tri (ROI) est prête, limiter la détection à l'intérieur
            # et préparer l'homographie pixel->cm
            if ROI_READY and len(ROI_POINTS_DISPLAY) == 4:
                h_img, w_img = img.shape[:2]
                # Mise à l'échelle des points ROI de l'affichage (600x300) vers l'image de traitement
                scale_x = float(w_img) / 600.0
                scale_y = float(h_img) / 300.0
                roi_pts_img = np.array(
                    [
                        [int(px * scale_x), int(py * scale_y)]
                        for (px, py) in ROI_POINTS_DISPLAY
                    ], dtype=np.int32
                )

                # Masque du polygone ROI
                roi_mask = np.zeros((h_img, w_img), dtype=np.uint8)
                cv2.fillPoly(roi_mask, [roi_pts_img], 255)

                # Appliquer le masque ROI à chaque masque couleur
                red_mask = cv2.bitwise_and(red_mask, red_mask, mask=roi_mask)
                green_mask = cv2.bitwise_and(green_mask, green_mask, mask=roi_mask)
                blue_mask = cv2.bitwise_and(blue_mask, blue_mask, mask=roi_mask)
                yellow_mask = cv2.bitwise_and(yellow_mask, yellow_mask, mask=roi_mask)

                # Calculer l'homographie pixel->cm à partir des 4 points
                # Ordre des points supposé: 4 coins du quadrilatère, à l'utilisateur de cliquer
                # dans le sens horaire ou anti-horaire. On les ordonne automatiquement.
                def order_points(pts):
                    pts = pts.reshape(4, 2)
                    s = pts.sum(axis=1)
                    diff = np.diff(pts, axis=1)
                    ordered = np.zeros((4, 2), dtype=np.float32)
                    ordered[0] = pts[np.argmin(s)]      # top-left
                    ordered[2] = pts[np.argmax(s)]      # bottom-right
                    ordered[1] = pts[np.argmin(diff)]   # top-right
                    ordered[3] = pts[np.argmax(diff)]   # bottom-left
                    return ordered

                src = order_points(roi_pts_img.astype(np.float32))
                dst = np.array([
                    [0.0, 0.0],
                    [ZONE_WIDTH_CM, 0.0],
                    [ZONE_WIDTH_CM, ZONE_HEIGHT_CM],
                    [0.0, ZONE_HEIGHT_CM]
                ], dtype=np.float32)

                # Homographie pour passer des pixels (img) aux cm (zone)
                try:
                    H = cv2.getPerspectiveTransform(src, dst)
                except Exception:
                    H = None
                global H_PIX_TO_CM
                H_PIX_TO_CM = H

            # Détection basée sur le plus grand contour (un seul contour par objet)
            def detect_largest_blob(mask_single, color_bgr, label):
                global POSX, POSY, REL_POSX_CM, REL_POSY_CM
                # Nettoyage pour stabilité
                kernel = np.ones((3, 3), np.uint8)
                mask_clean = cv2.morphologyEx(mask_single, cv2.MORPH_OPEN, kernel, iterations=1)
                mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel, iterations=2)

                # Trouver contours externes uniquement pour éviter les doublons/holes
                contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not contours:
                    return 0

                # Choisir le plus grand contour par aire
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                if area < 300:
                    return 0

                # Approximations pour lisser la forme
                peri = cv2.arcLength(largest, True)
                approx = cv2.approxPolyDP(largest, 0.01 * peri, True)

                # Calcul du centre via moments; fallback cercle englobant si m00==0
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    (cx, cy), _ = cv2.minEnclosingCircle(largest)
                    cx, cy = int(cx), int(cy)

                # Dessiner le contour exact et le centre
                cv2.drawContours(imgContour, [approx], -1, color_bgr, 2)
                cv2.circle(imgContour, (cx, cy), 5, (255, 255, 255), -1)
                cv2.putText(imgContour, f"{label}", (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

                # Mettre à jour POSX/POSY (pixels image traitée)
                POSX = cx
                POSY = cy

                # Calculer la position en cm si homographie dispo
                REL_POSX_CM = None
                REL_POSY_CM = None
                if H_PIX_TO_CM is not None:
                    pt = np.array([[float(cx), float(cy), 1.0]], dtype=np.float32).T
                    mapped = H_PIX_TO_CM @ pt
                    denom = float(mapped[2][0]) if mapped[2][0] != 0 else 1.0
                    Xcm = float(mapped[0][0]) / denom
                    Ycm = float(mapped[1][0]) / denom
                    REL_POSX_CM = Xcm
                    REL_POSY_CM = Ycm

                return int(area)

            # Exécuter sur chaque couleur
            areas = {
                'red': detect_largest_blob(red_mask, (0, 0, 255), 'Red'),
                'green': detect_largest_blob(green_mask, (0, 255, 0), 'Green'),
                'blue': detect_largest_blob(blue_mask, (255, 0, 0), 'Blue'),
                'yellow': detect_largest_blob(yellow_mask, (0, 255, 255), 'Yellow')
            }
        except Exception as e:
            print(f"Error in color detection: {e}")
            return None

        return imgContour

    except Exception as e:
        print(f"Error in getImageCoordinates: {e}")
        return None

# Initialize camera with proper settings
def init_camera():
    global cap, USE_SNAPSHOT, CAM_URL
    max_retries = 3
    retry_count = 0
    
    while retry_count < max_retries:
        try:
            if cap is not None:
                cap.release()
            
            # Handle URL vs local device index differently
            if isinstance(CAMERANUMBER, str):
                base = CAMERANUMBER.rstrip('/')
                candidate_urls = [
                    base,
                    base + '/video',
                    base + '/stream',
                    base + '/mjpeg',
                    base + '/mjpeg/1',
                    base + ('?action=stream' if '?' not in base else ''),
                ]
                opened = False
                last_err = None
                for url in candidate_urls:
                    if not url:
                        continue
                    try:
                        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
                        if not cap.isOpened():
                            if cap is not None:
                                cap.release()
                            continue
                        # Do NOT set FOURCC/FPS for HTTP streams
                        ret, frame = cap.read()
                        if not ret or frame is None:
                            if cap is not None:
                                cap.release()
                            continue
                        opened = True
                        break
                    except Exception as ee:
                        last_err = ee
                        if cap is not None:
                            cap.release()
                        continue
                if not opened:
                    # Snapshot fallback (e.g., Android IP Webcam /shot.jpg)
                    snapshot_candidates = [
                        base + '/shot.jpg',
                        base + '/photo.jpg',
                        base + '/image.jpg',
                    ]
                    snap_ok = False
                    for surl in snapshot_candidates:
                        try:
                            with urllib.request.urlopen(surl, timeout=3) as resp:
                                jpg = resp.read()
                            test = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if test is not None:
                                CAM_URL = surl
                                USE_SNAPSHOT = True
                                print(f"Camera initialized via HTTP snapshot: {surl}")
                                snap_ok = True
                                break
                        except Exception as ee2:
                            last_err = ee2
                            continue
                    if not snap_ok:
                        raise Exception(f"Failed to open camera URL. Last error: {last_err}")
            else:
                # Local device index
                cap = cv2.VideoCapture(CAMERANUMBER)
                if not cap.isOpened():
                    raise Exception("Failed to open camera device")
                # Set camera properties for local devices only
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                cap.set(cv2.CAP_PROP_FPS, 30)
                # Test capture
                ret, frame = cap.read()
                if not ret or frame is None:
                    raise Exception("Failed to read test frame")
                
            print("Camera initialized successfully")
            return True
            
        except Exception as e:
            print(f"Camera initialization attempt {retry_count + 1} failed: {str(e)}")
            if cap is not None:
                cap.release()
            retry_count += 1
            time.sleep(2)
    
    return False

# Initialize camera at startup
if not init_camera():
    print("Failed to initialize camera after multiple attempts")
    exit(1)

def getColor():
    """
    Renvoie la couleur détectée à la position courante de l'objet.
    - Ne renvoie rien tant qu'aucune position POSX/POSY n'est détectée.
    - Utilise l'image HSV globale calculée dans getImageCoordinates() pour éviter
      les faux positifs dus aux annotations (rectangles/texte) sur l'image.
    """
    # Mettre à jour la détection et l'image HSV globale
    img = getImageCoordinates()
    if img is None:
        return None
        
    # Vérifier si une position a été détectée lors de getImageCoordinates()
    if POSX is None or POSY is None:
        return None

    # S'assurer que l'image HSV globale est disponible
    if hsv_img is None:
        return None

    # Échantillonner la couleur au centre détecté (en évitant les annotations)
    h, w = hsv_img.shape[:2]
    x = max(0, min(w - 1, POSX))
    y = max(0, min(h - 1, POSY))
    pixel = hsv_img[y, x]

    # Vérifier les intervalles HSV pour déterminer la couleur
    if (red_lower <= pixel).all() and (pixel <= red_upper).all():
            return "red"
    if (green_lower <= pixel).all() and (pixel <= green_upper).all():
            return "green"
    if (blue_lower <= pixel).all() and (pixel <= blue_upper).all():
            return "blue"
    if (yellow_lower <= pixel).all() and (pixel <= yellow_upper).all():
            return "yellow"
    
    return None

if __name__ == "__main__":
   while True:
        try:
            success, img = cap.read()
        except Exception as e:
            img = cv2.imread("assets/brain2.png")
        cv2.imshow("window", img)
        img = crop(img)
        img = cv2.flip(img, 0)
        imgContour = img.copy()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv_img, red_lower, red_upper)
        green_mask = cv2.inRange(hsv_img, green_lower, green_upper)
        blue_mask = cv2.inRange(hsv_img, blue_lower, blue_upper)
        yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)

        # Apply dilation to masks
        kernel = np.ones((5, 5), "uint8")
        red_mask = cv2.dilate(red_mask, kernel)
        green_mask = cv2.dilate(green_mask, kernel)
        blue_mask = cv2.dilate(blue_mask, kernel)
        yellow_mask = cv2.dilate(yellow_mask, kernel)

        # colorMask = cv2.inRange(hsv_img, lower, upper)
        detect_color(red_mask, imgContour, "Red", (0, 0, 255))
        detect_color(green_mask, imgContour, "Green", (0, 255, 0))
        detect_color(blue_mask, imgContour, "Blue", (255, 0, 0))
        detect_color(yellow_mask, imgContour, "Yellow", (0, 255, 255))
        cv2.imshow("Result", imgContour)
        cv2.setMouseCallback('Result', mouseClickHandler)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

