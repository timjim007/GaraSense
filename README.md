# Smart Environmental & Parking Assist System

A real-time embedded system built on the STM32F407 that monitors environmental conditions and provides distance-based LED feedback for parking assistance.
Designed with FreeRTOS and an Active Object (event-driven) architecture for responsive, modular operation.

---

##  Demo
![alt text](Demo/Demo.mp4)
- More Demo images in Demo/
---

##  Features

- **Ultrasonic Distance Sensing** – HCSR04 sensor with WS2812 RGB LED feedback for visual parking guidance.
- **Environmental Monitoring** – DHT11 temperature/humidity and MQ-135 air-quality sensors.
- **OLED User Interface** – Real-time readings on a 0.96″ I²C display.
- **Event-Driven FreeRTOS Architecture** – Active objects communicate through queues for clean concurrency.


---

## System Architecture

### Hardware

- **STM32F407G-DISC1** development board  
- **DHT11** – Temperature & humidity sensor  
- **MQ135** – Gas/CO₂ sensor  
- **HC-SR04** – Ultrasonic distance sensor  
- **SSD1306 128×64 OLED** (I²C)  
- **MicroSD adapter**

> **Pin Configuration:**  
> All pin assignments are specified in `garasense.ioc`.  
> Open the file in **STM32CubeMX** to view or modify connections.


### Software Architecture

The firmware is divided into clear layers:

- **ECUAL Folder** – Hardware Abstraction Layer for all peripheral drivers.  
- **Application Folder** – Defines the **Active Objects** and their behaviors.  
- **Main** – Instantiates active objects; **FreeRTOS** handles scheduling.

### Design Highlights
- **Event-driven, object-oriented architecture** built on FreeRTOS.
- **No mutexes or semaphores**: avoids race conditions and synchronization issues.
- Each **Active Object** encapsulates:
  - Private data
  - Private queue
  - Private thread

### Event Structure
```c
EventSignal eventsig;
EventDestination eventdest;
EventPayload eventpayload;
```

### Active Objects

| Object          | Role                                                                 |
|-----------------|----------------------------------------------------------------------|
| **Broker**      | Acts as an event bus/router to direct events to their destinations |
| **Display**     | Updates the OLED screen with current readings and messages   |
| **DistanceSensor** | Tracks vehicle distance using the ultrasonic sensor and updates LED |
| **EnvMonitor**  | Monitors temperature, humidity, and gas/CO₂ levels using DHT11 and MQ-135 |


## Debugging
A dedicated **FreeRTOS task** continuously monitors:
- **Queue sizes** for each active object  
- **Available stack size** per active object  

This makes it easier to detect memory or scheduling issues during runtime.

---

## Known Issues
- **SD Card Logging**  
  Attempting to log data to the MicroSD card currently causes the system to block because the library uses a **blocking SPI** implementation.  
  - **Planned Fix:** Switch to **non-blocking SPI (DMA)** for logging.

---

## Future Improvements
- Integrate an **ESP32 Wi-Fi module** for cloud logging & remote monitoring  
- Design and fabricate a **custom PCB**  
- Create a **3D-printed or fabricated enclosure**  
- Implement **non-blocking SPI** for reliable SD card logging

---

## Getting Started
1. **Clone the Repository**
   ```bash
   git clone https://github.com/<your-username>/<repo-name>.git


2. **Open the Project**  
   Launch `garasense.ioc` in **STM32CubeMX** to review and configure the hardware setup.

3. **Build the Firmware**  
   Compile the code using your preferred STM32 IDE (e.g., **STM32CubeIDE**).

4. **Flash and Run**  
   Upload the compiled firmware to the **STM32F407G-DISC1** board and start the system.


## License

MIT License — feel free to fork, modify, and build on top of it.

## Resources

The following references were helpful in the design and development of this project:

- [state-machine.com](https://www.state-machine.com/) – Concepts on event-driven programming and active object patterns.  
- [deepbluembedded.com](https://deepbluembedded.com/) – STM32 tutorials and embedded systems best practices.