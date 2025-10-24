import tkinter as tk
import servocontroller
import threading
import cv2
from PIL import Image, ImageTk
import objectdetector
import time
import numpy as np


WIDTH = 1360
HEIGHT = 768
BACKGROUND = "#1B2430"
FONTCOLOR = "#D6D5A8"
OBJX = None
OBJY = None
BUTTONFONTSIZE = 15
CONNECTED = False


def hello(*args, **kwargs):
    print(WIDTH)

def cvtImage(cv2image):
    cv2image = cv2.cvtColor(cv2image, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(cv2image)
    img = img.resize((580, 508))

    # Convert image to PhotoImage
    imgtk = ImageTk.PhotoImage(image=img)
    return imgtk

class GUI(tk.Tk):
    def __init__(self):
        super().__init__()  # Initiating Super Method
        self.state("zoomed")
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.title("Robotic Arm Controller")
        self.config(background=BACKGROUND)
        self.updateScreenGeometry()

        self.pages = {}
        self.raised = []
        for P in (WelcomePage, PickAndPlacePage, ColorSortingPage, ManualControlPage):
            page = P(self)
            self.pages[P] = page

        self.showPage(WelcomePage)

    def showPage(self, page):
        self.raised = []
        self.raised.append(page)
        frame = self.pages[page]
        frame.tkraise()

    def updateScreenGeometry(self):
        global WIDTH, HEIGHT
        WIDTH = self.winfo_screenwidth()
        HEIGHT = self.winfo_screenheight()

class Page(tk.Frame):
    def __init__(self, containerWindow):
        super().__init__(containerWindow, bg=BACKGROUND)
        self.container = containerWindow
        self.grid(row=0, column=0, sticky="nsew")

class TextLabel(tk.Label):
    def __init__(self, page, text, fontsize=20):
        super().__init__(
            page,
            fg=FONTCOLOR,
            text=text,
            font=('Arial', fontsize),
            bg=BACKGROUND
        )

class ImageLabel(tk.Label):
    def __init__(self, page, location, size, bg=BACKGROUND):
        imageRaw = Image.open(location)
        imageRaw = imageRaw.resize(size)
        self.image = ImageTk.PhotoImage(imageRaw)  # Avoid garbage collection
        super().__init__(page, bg=BACKGROUND, image=self.image)

class WebcamLabel(tk.Label):
    def __init__(self, parent, width, height):
        super().__init__(parent)
        self.width = width
        self.height = height

class ButtonLabel(tk.Button):
    def __init__(self, page, text, fontsize=15, command=hello, width=50, borderwidth=3, state=tk.NORMAL):
        super().__init__(page,
                         text=text,
                         font=('Arial', fontsize),
                         relief=tk.RAISED,
                         bg=BACKGROUND,
                         fg=FONTCOLOR,
                         width=width,
                         borderwidth=borderwidth,
                         command=command,
                         activebackground=FONTCOLOR,
                         activeforeground=BACKGROUND,
                         state=state
                         )

class SliderLabel(tk.Scale):
    def __init__(self, page, from_, to, resolution, length=300, command=hello):
        super().__init__(
            page,
            from_=from_,
            to=to,
            orient=tk.HORIZONTAL,
            length=length,
            background=BACKGROUND,
            fg=FONTCOLOR,
            resolution=resolution,
            command=command
        )

class WelcomePage(Page):
    def __init__(self, container):
        super().__init__(container)

        text = TextLabel(self, text="Welcome to RobotAC", fontsize=40)
        text.pack(fill="x")

        headingImage = ImageLabel(
            self,
            location="../assets/brain.png",
            size=(300, 300)
        )
        headingImage.pack(fill='x', pady=10)

        # About section
        aboutText = TextLabel(
            self,
            text="RobotAC is a robotic arm controller software MADE BY MOHAMMED EL KASSOIRI"
        )
        aboutText.pack(fill="x", pady=20, padx=20)

        self.pickAndPlaceButton = ButtonLabel(
            self,
            text="Pick & Place Mode",
            command=lambda: self.container.showPage(PickAndPlacePage)
        )
        self.pickAndPlaceButton.pack()

        self.manualButton = ButtonLabel(
            self,
            text="Manual Mode",
            command=lambda: self.container.showPage(ManualControlPage)
        )
        self.manualButton.pack()

        self.colorSortingButton = ButtonLabel(
            self,
            text="Color Sorting Mode",
            command=lambda: self.container.showPage(ColorSortingPage)

        )
        self.colorSortingButton.pack()

        # Connection controls
        connectionFrame = tk.Frame(self, bg=BACKGROUND)
        connectionFrame.pack(pady=10)

        self.portEntry = tk.Entry(connectionFrame, width=10, font=("Arial", 20))
        self.portEntry.insert(0, servocontroller.PORT)
        self.portEntry.pack(side="left", padx=10)

        self.connectButton = ButtonLabel(
            connectionFrame,
            text="Connect",
            width=20,
            command=self.connectToArduino
        )
        self.connectButton.pack(side="left", padx=10)

        self.connectionStatus = TextLabel(
            connectionFrame,
            text="Not Connected",
            fontsize=15
        )
        self.connectionStatus.pack(side="left", padx=10)

        self.exitButton = ButtonLabel(
            self,
            text="Exit",
            command=self.container.destroy
        )
        self.exitButton.pack()

        self.disableButtons()

    def connectToArduino(self):
        global CONNECTED
        port = self.portEntry.get()
        self.connectionStatus.configure(text="Connecting...")
        self.container.update()
        CONNECTED = servocontroller.connect(port)

        if (CONNECTED):
            self.connectionStatus.configure(text=f"Connected to {port}")
            self.enableButtons()
        else:
            self.connectionStatus.configure(text="Not Connected")
            self.disableButtons()

    def disableButtons(self):
        self.pickAndPlaceButton["state"] = tk.DISABLED
        self.colorSortingButton["state"] = tk.DISABLED
        self.manualButton["state"] = tk.DISABLED
        pass

    def enableButtons(self):
        self.pickAndPlaceButton["state"] = tk.NORMAL
        self.colorSortingButton["state"] = tk.NORMAL
        self.manualButton["state"] = tk.NORMAL

class PickAndPlacePage(Page):
    def __init__(self, container):
        super().__init__(container)
        pickAndPlaceHeader = TextLabel(
            self,
            text="PICK & PLACE",
            fontsize=20,
        )
        pickAndPlaceHeader.pack(fill='x')
        self.setup_ui()

        self.videoFeed = WebcamLabel(
            self,
            width=600,
            height=580,
        )
        self.videoFeed.place(x=580, y=70)
        self.videoFeed.bind("<Button-3>", self.selectPosition)


    def update_frame(self,image):
        self.display_image(image, self.videoFeed)

    def display_image(self, img, widget):
        imgtk = ImageTk.PhotoImage(Image.fromarray(img))
        widget.imgtk = imgtk
        widget.config(image=imgtk)

    def setup_ui(self):
        self.backButton = ButtonLabel(
            self,
            text="Back",
            command=lambda: self.container.showPage(WelcomePage)
        )
        self.backButton.pack(pady=5, padx=10, side="bottom", anchor="w")

        self.placeButton = ButtonLabel(
            self,
            text="Place",
            command=lambda: threading.Thread(
                target=self.placeObjectGUI).start()
        )
        self.placeButton.pack(pady=2, padx=10, side="bottom", anchor="w")

        self.pickButton = ButtonLabel(
            self,
            text="Pick",
            command=lambda: threading.Thread(target=self.pickObjectGUI).start()
        )
        self.pickButton.pack(pady=2, padx=10, side="bottom", anchor="w")

        self.autoButton = ButtonLabel(
            self,
            text="Auto Pick Up",
            command=lambda: threading.Thread(
                target=self.autoPickObject).start()
        )
        self.autoButton.pack(pady=2, padx=10, side="bottom", anchor="w")

        self.xSlider = SliderLabel(self,
                              from_=-12,
                              to=12,
                              length=300,
                              resolution=0.01,
                              command=self.sliderUpdate
                              )
        self.xSlider.place(x=40, y=100)
        self.xSliderLabel = TextLabel(self, "X")
        self.xSliderLabel.place(x=10, y=105)
    

        self.ySlider = SliderLabel(self,
                              from_=0,
                              to=18,
                              command=self.sliderUpdate,
                              resolution=0.01
                              )
        self.ySlider.set(10)
        self.ySlider.place(x=40, y=150)
        self.ySliderLabel = TextLabel(self,
                                 text="Y",
                                 )
        self.ySliderLabel.place(x=10, y=155)

        self.coordinateLabel = TextLabel(self,
                                    text="( X , Y )",
                                    )
        self.coordinateLabel.place(x=10, y=255)
        self.objectPositionLabel = TextLabel(self,
                                        text="( X , Y )",
                                        )
        self.objectPositionLabel.place(x=10, y=355)

    def placeObjectGUI(self):
        servocontroller.placeObject([self.xSlider.get(), self.ySlider.get()])

    def pickObjectGUI(self):
        servocontroller.pickObject([self.xSlider.get(), self.ySlider.get()])

    def sliderUpdate(self, *args):
        self.coordinateLabel['text'] = f'Manual Coordinates: ( {self.xSlider.get()} , {self.ySlider.get()} )'

    def autoPickObject(self):
        global OBJX, OBJY
        OBJX = OBJX
        servocontroller.pickObject([OBJX, OBJY])
        time.sleep(0.5)
        servocontroller.placeObject([self.xSlider.get(), self.ySlider.get()])

    def calculateActualPosition(self, objX, objY):
        xx = (((12+12)/475)*objX) - 12.1
        yy = (18) - ((18)/280 * objY)
        try:
            self.objectPositionLabel.configure(
                text=f"Object Detected At: ( {round(OBJX, 2)}, {round(OBJY, 2)} )")
        except Exception as e:
            print(e)
        return xx, yy

    def showPositionIndicator(self, img):
        try:
            xPix = int((self.xSlider.get() + 12) * (600/24))
            yPix = int(280-((self.ySlider.get()) * (280/18)))
            cv2.rectangle(img, (xPix, yPix),
                          (xPix+5, yPix+5), (0, 0, 120), thickness=10)
        except Exception as e:
            cv2.rectangle(img, (10, 10),
                          (20, 20), (120, 0, 0), 2)
            print(e)

    def selectPosition(self, event):
        x, y = self.calculateActualPosition(event.x, event.y)
        self.xSlider.set(x)
        self.ySlider.set(y)

class ColorSortingPage(Page):
    def __init__(self, container):
        super().__init__(container)
        colorSortingHeader = TextLabel(
            self,
            text="COLOR SORTING",
            fontsize=20,
        )
        colorSortingHeader.pack(fill='x')
        self.setup_ui()

        self.videoFeed = WebcamLabel(
            self,
            width=600,
            height=580,
        )
        self.videoFeed.place(x=580, y=70)
        # Bind ROI clicks only in ColorSortingPage
        self.videoFeed.bind("<Button-1>", self.on_image_click)
        # Cache des positions cibles pour thread-safety
        self._targets_cache = {
            'red': (0.0, 0.0),
            'green': (0.0, 0.0),
            'yellow': (0.0, 0.0),
            'blue': (0.0, 0.0),
        }

    def update_frame(self, image):
        # Draw target points for each color
        self.drawColorTargets(image)
        # Dessiner la zone de tri si des points ont été définis
        try:
            if len(objectdetector.ROI_POINTS_DISPLAY) > 0:
                pts = np.array(objectdetector.ROI_POINTS_DISPLAY, dtype=np.int32)
                cv2.polylines(image, [pts], isClosed=(len(pts) == 4), color=(255, 255, 255), thickness=2)
                for (px, py) in objectdetector.ROI_POINTS_DISPLAY:
                    cv2.circle(image, (int(px), int(py)), 4, (255, 255, 255), -1)
        except Exception:
            pass
        self.display_image(image, self.videoFeed)

    def drawColorTargets(self, img):
        # Draw red target
        redX = int((self.redXSlider.get() + 12) * (600/24))
        redY = int(280-((self.redYSlider.get()) * (280/18)))
        cv2.circle(img, (redX, redY), 5, (255, 0, 0), -1)  # Red circle
        cv2.putText(img, "Red", (redX + 10, redY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        # Update cache on main thread
        self._targets_cache['red'] = (float(self.redXSlider.get()), float(self.redYSlider.get()))

        # Draw green target
        greenX = int((self.greenXSlider.get() + 12) * (600/24))
        greenY = int(280-((self.greenYSlider.get()) * (280/18)))
        cv2.circle(img, (greenX, greenY), 5, (0, 255, 0), -1)  # Green circle
        cv2.putText(img, "Green", (greenX + 10, greenY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        self._targets_cache['green'] = (float(self.greenXSlider.get()), float(self.greenYSlider.get()))

        # Draw yellow target
        yellowX = int((self.yellowXSlider.get() + 12) * (600/24))
        yellowY = int(280-((self.yellowYSlider.get()) * (280/18)))
        cv2.circle(img, (yellowX, yellowY), 5, (255, 255, 0), -1)  # Yellow circle
        cv2.putText(img, "Yellow", (yellowX + 10, yellowY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        self._targets_cache['yellow'] = (float(self.yellowXSlider.get()), float(self.yellowYSlider.get()))

        # Draw blue target
        blueX = int((self.blueXSlider.get() + 12) * (600/24))
        blueY = int(280-((self.blueYSlider.get()) * (280/18)))
        cv2.circle(img, (blueX, blueY), 5, (0, 0, 255), -1)  # Blue circle
        cv2.putText(img, "Blue", (blueX + 10, blueY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        self._targets_cache['blue'] = (float(self.blueXSlider.get()), float(self.blueYSlider.get()))

    def display_image(self, img, widget):
        imgtk = ImageTk.PhotoImage(Image.fromarray(img))
        widget.imgtk = imgtk
        widget.config(image=imgtk)

    def on_image_click(self, event):
        try:
            objectdetector.add_roi_point_display(event.x, event.y)
        except Exception:
            pass

    def setup_ui(self):
        self.backButton = ButtonLabel(
            self,
            text="Back",
            command=lambda: self.container.showPage(WelcomePage)
        )
        self.backButton.pack(pady=5, padx=10, side="bottom", anchor="w")

        self.autoButton = ButtonLabel(
            self,
            text="Auto Color Sort",
            command=lambda: threading.Thread(
                target=self.autoColorSort).start()
        )
        self.autoButton.pack(pady=2, padx=10, side="bottom", anchor="w")


        self.redXSlider = SliderLabel(self,
                              from_=-12,
                              to=12,
                              length=300,
                              resolution=0.01,
                              )
        self.redXSlider.place(x=120, y=100)
        self.redXSliderLabel = TextLabel(self, "Red X")
        self.redXSliderLabel.place(x=10, y=105)

        self.redYSlider = SliderLabel(self,
                              from_=0,
                              to=18,
                              resolution=0.01
                              )
        self.redYSlider.set(10)
        self.redYSlider.place(x=120, y=150)
        self.redYSliderLabel = TextLabel(self,
                                 text="Red Y",
                                 )
        self.redYSliderLabel.place(x=10, y=155)

        # Green position sliders
        self.greenXSlider = SliderLabel(self,
                              from_=-12,
                              to=12,
                              length=300,
                              resolution=0.01,
                              )
        self.greenXSlider.place(x=120, y=200)
        self.greenXSliderLabel = TextLabel(self, "Green X")
        self.greenXSliderLabel.place(x=10, y=205)

        self.greenYSlider = SliderLabel(self,
                              from_=0,
                              to=18,
                              resolution=0.01
                              )
        self.greenYSlider.set(10)
        self.greenYSlider.place(x=120, y=250)
        self.greenYSliderLabel = TextLabel(self,
                                 text="Green Y",
                                 )
        self.greenYSliderLabel.place(x=10, y=255)

        # Yellow position sliders
        self.yellowXSlider = SliderLabel(self,
                              from_=-12,
                              to=12,
                              length=300,
                              resolution=0.01,
                              )
        self.yellowXSlider.place(x=120, y=300)
        self.yellowXSliderLabel = TextLabel(self, "Yellow X")
        self.yellowXSliderLabel.place(x=10, y=305)

        self.yellowYSlider = SliderLabel(self,
                              from_=0,
                              to=18,
                              resolution=0.01
                              )
        self.yellowYSlider.set(10)
        self.yellowYSlider.place(x=120, y=350)
        self.yellowYSliderLabel = TextLabel(self,
                                 text="Yellow Y",
                                 )
        self.yellowYSliderLabel.place(x=10, y=355)

        # Blue position sliders
        self.blueXSlider = SliderLabel(self,
                              from_=-12,
                              to=12,
                              length=300,
                              resolution=0.01
                              )
        self.blueXSlider.place(x=120, y=400)
        self.blueXSliderLabel = TextLabel(self, "Blue X")
        self.blueXSliderLabel.place(x=10, y=405)

        self.blueYSlider = SliderLabel(self,
                              from_=0,
                              to=18,
                              resolution=0.01
                              )
        self.blueYSlider.set(10)
        self.blueYSlider.place(x=120, y=450)
        self.blueYSliderLabel = TextLabel(self,
                                 text="Blue Y",
                                 )
        self.blueYSliderLabel.place(x=10, y=455)

        self.objectPositionLabel = TextLabel(self,
                                            text="( X , Y )",
                                            )
        self.objectPositionLabel.place(x=10, y=520)

    def autoColorSort(self):
        while True:
            # 1) Détection d'abord
            color = objectdetector.getColor()
            posx, posy = objectdetector.getPos()

            if color is None or posx is None or posy is None:
                # Aucun objet détecté: rester en position initiale et attendre
                time.sleep(0.5)
                continue

            if color not in ["red", "green", "yellow", "blue"]:
                time.sleep(0.5)
                continue

            # Convertir la position image -> coordonnées robot
            pick_x, pick_y = self.calculateActualPosition(posx, posy)

            # 2) Pick
            servocontroller.pickObject([pick_x, pick_y])
            time.sleep(0.3)

            # 3) Place selon la couleur (le retour à la position initiale
            #    est géré à la fin de placeObject dans le contrôleur)
            if color == "red":
                tx, ty = self._targets_cache['red']
                servocontroller.placeObject([tx, ty])
            elif color == "green":
                tx, ty = self._targets_cache['green']
                servocontroller.placeObject([tx, ty])
            elif color == "yellow":
                tx, ty = self._targets_cache['yellow']
                servocontroller.placeObject([tx, ty])
            elif color == "blue":
                tx, ty = self._targets_cache['blue']
                servocontroller.placeObject([tx, ty])

            # 4) Une fois revenu à la position initiale, on recommence la détection
            time.sleep(0.3)

    def calculateActualPosition(self, objX, objY):
        xx = (((12+12)/475)*objX) - 12.1
        yy = (18) - ((18)/280 * objY)
        try:
            self.objectPositionLabel.configure(
                text=f"Object Detected At: ( {round(OBJX, 2)}, {round(OBJY, 2)} )")
        except Exception as e:
            print(e)
        return xx, yy

class ManualControlPage(Page):
    def __init__(self, container):
        super().__init__(container)

        manualControlHeader = TextLabel(
            self,
            text="Manual Control",
        )
        manualControlHeader.pack(fill='x')
        self.setup()

        self.videoFeed1 = WebcamLabel(
            self,
            width=600,
            height=600,
        )
        self.videoFeed1.place(x=600, y=70)

    def update_frame1(self,image):
        self.display_image1(image, self.videoFeed1)
        
    def display_image1(self, img, widget):
        imgtk1 = ImageTk.PhotoImage(Image.fromarray(img))
        widget.imgtk = imgtk1
        widget.config(image=imgtk1)

    def setup(self):
        self.backButton = ButtonLabel(
            self,
            text="Back",
            command=lambda: self.container.showPage(WelcomePage)
        )
        self.backButton.pack(pady=5, padx=10, side="bottom", anchor="w")

        self.baseServoSlider = SliderLabel(self, 
                                           from_=0, 
                                           to=270,
                                      resolution=0.01,
                                      command=self.manualControl
                                      )
        self.baseServoSlider.set(90)
        self.baseServoSlider.place(x=200, y=100)
        self.baseServoSliderLabel = TextLabel(self,
                                         text="Base",
                                         )
        self.baseServoSliderLabel.place(x=50, y=100)
        self.shoulderServoSlider = SliderLabel(self,
                                          from_=0,
                                          to=270,
                                          resolution=0.01,
                                          command=self.manualControl
                                          )
        self.shoulderServoSlider.set(210)
        self.shoulderServoSlider.place(x=200, y=200)
        self.shoulderServoSliderLabel = TextLabel(self,
                                             text="Shoulder",
                                             )
        self.shoulderServoSliderLabel.place(x=50, y=200)

        self.elbowServoSlider = SliderLabel(self,
                                       from_=0,
                                       to=270,
                                       resolution=0.01,
                                       command=self.manualControl
                                       )
        self.elbowServoSlider.set(45)
        self.elbowServoSlider.place(x=200, y=300)
        self.elbowServoSliderLabel = TextLabel(self,
                                          text="Elbow",
                                          )
        self.elbowServoSliderLabel.place(x=50, y=300)

        self.grabberServoSlider = SliderLabel(self,
                                         from_=50,
                                         to=100,
                                         resolution=0.01,
                                         command=self.manualControl
                                         )
        self.grabberServoSlider.set(120)
        self.grabberServoSlider.place(x=200, y=400)
        self.grabberServoSliderLabel = TextLabel(self,
                                            text="Grabber",
                                            )
        self.grabberServoSliderLabel.place(x=50, y=400)

    def manualControl(self, *args):
        servo1Angle = self.baseServoSlider.get()
        servo2Angle = self.shoulderServoSlider.get()
        servo3Angle = self.elbowServoSlider.get()
        servo4Angle = self.grabberServoSlider.get()
        try:
            servocontroller.guiControl(
                servo1Angle, servo2Angle, servo3Angle, servo4Angle)
        except Exception as e:
            pass

def update_loop():
    try:
        cv2image = objectdetector.getImageCoordinates()
        if cv2image is None:
            window.after(10, update_loop)  # Retry after 10ms
            return

        objX, objY = objectdetector.getPos()
        if objX is not None:
            global OBJX, OBJY
            if window.raised[0] == PickAndPlacePage:
                OBJX, OBJY = window.pages[PickAndPlacePage].calculateActualPosition(objX, objY)
            elif window.raised[0] == ColorSortingPage:
                OBJX, OBJY = window.pages[ColorSortingPage].calculateActualPosition(objX, objY)

        # Use the same image processing as PickAndPlace
        cv2image = cv2.resize(cv2image, (600, 300))
        cv2image = cv2.cvtColor(cv2image, cv2.COLOR_BGR2RGB)
        
        # Update position indicator for current mode
        if window.raised[0] == PickAndPlacePage:
            window.pages[PickAndPlacePage].showPositionIndicator(cv2image)
            window.pages[PickAndPlacePage].update_frame(cv2image)
        elif window.raised[0] == ColorSortingPage:
            window.pages[ColorSortingPage].update_frame(cv2image)
        elif window.raised[0] == ManualControlPage:
            window.pages[ManualControlPage].update_frame1(cv2image)

    except Exception as e:
        print(f"Error in update loop: {e}")
    
    window.after(10, update_loop)  # schedule next update

# lancement correct
window = GUI()
update_loop()
window.mainloop()