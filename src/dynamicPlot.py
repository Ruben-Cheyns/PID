import serial
import sys
from matplotlib import pyplot as plt
import pandas as pd
import threading
import queue
import time

# Initialize serial connection with error handling
ser = None
try:
    # VEX robots typically use 115200 baud rate
    # Adjust if your robot uses a different rate
    BAUD_RATE = 115200
    ser = serial.Serial('COM7', baudrate=BAUD_RATE, timeout=0.1)
    print(f"Successfully opened COM7 at {BAUD_RATE} baud")
except serial.SerialException as e:
    print(f"Error opening COM7: {e}")
    print("Available serial ports:")
    from serial.tools import list_ports
    ports = list_ports.comports()
    if ports:
        for port in ports:
            print(f"  - {port.device}: {port.description}")
    else:
        print("  No serial ports found")
    sys.exit(1) 

# Column names for the plotter data: {time, prop, deriv, integral, output, desiredValue, angle}
dataColumns = ['time', 'proportional', 'derivative', 'integral', 'output', 'desiredValue', 'angle']
angleCols = ['desiredValue', 'angle']
constantCols = ['proportional', 'derivative', 'integral', 'output']
timeCol = 'time'

# Thread-safe queue for passing data from serial reader to main thread
data_queue = queue.Queue()
stop_event = threading.Event()

def preProcessData(incomingDatum):
    '''
    Parse data in format: {timestamp,value1,value2,...}
    Mimics the serial plotter's preProcessData function
    '''
    try:
        incomingDatum = incomingDatum.decode('utf-8')
        incomingDatum = incomingDatum.strip('\n')
        
        # Extract data between { and }
        try:
            incomingDatum = incomingDatum[incomingDatum.index('{')+len('{'):incomingDatum.index('}')] 
        except:
            incomingDatum = ''
        
        # Split by comma and convert to floats
        values = incomingDatum.split(',')
        values = [float(v.strip()) for v in values if v.strip()]
        return values
    except Exception as e:
        print(f"[PARSE ERROR] {e}")
        return []

def serial_reader():
    """Background thread that reads from serial port in plotter format"""
    data_lines = []
    collecting = False
    
    try:
        while not stop_event.is_set():
            try:
                readout = ser.readline()
                
                if not readout:
                    time.sleep(0.01)
                    continue
                
                # Debug: Print raw bytes received
                print(f"[SERIAL DEBUG] Raw: {readout}")
                
                line = readout.decode('utf-8').strip()
                print(f"[SERIAL DEBUG] Decoded: '{line}'")
                
                # Check for START marker
                if "{START}" in line:
                    collecting = True
                    data_lines = []
                    print("[SERIAL] START marker received, collecting data...")
                    continue
                
                # Check for STOP marker
                if "{STOP}" in line and collecting:
                    if data_lines:
                        data_queue.put(data_lines.copy())
                        print(f"[SERIAL] STOP marker received, data queued ({len(data_lines)} points)")
                    data_lines = []
                    collecting = False
                    continue
                
                # Collect data lines: {time,prop,deriv,integral,output,desiredValue,angle}
                if collecting and line.startswith('{') and line.endswith('}'):
                    try:
                        values = preProcessData(readout)
                        if len(values) == 7:  # Should have 7 values
                            data_lines.append(values)
                            print(f"[SERIAL] Data point {len(data_lines)}: {values}")
                        else:
                            print(f"[SERIAL WARNING] Expected 7 values, got {len(values)}: {values}")
                    except Exception as e:
                        print(f"[SERIAL ERROR] Failed to parse: {e}")
                        
            except UnicodeDecodeError as e:
                print(f"[SERIAL DEBUG] Decode error: {e}")
                continue
            except Exception as e:
                print(f"[SERIAL ERROR] Reader error: {e}")
                time.sleep(0.1)
    except Exception as e:
        print(f"[SERIAL ERROR] FATAL: {e}")
        raise

try:
    # Enable interactive mode and create figure
    plt.ion()
    fig, axs = plt.subplots(2, 6, sharex=True, sharey=True)
    plt.tight_layout()
    
    # Show the figure so it's responsive
    fig.show()
    plt.pause(0.001)

except Exception as e:
    print(f"Error setting up matplotlib: {e}")
    sys.exit(1)

# Start serial reader in background thread
reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()
print(f"[STARTUP] Serial reader thread started. Waiting for data...")
print(f"[STARTUP] Thread is alive: {reader_thread.is_alive()}")

# Give the robot time to boot and start sending
time.sleep(2)
print(f"[STARTUP] Ready to receive data")

i = 0

try:
    while True:
        try:
            # Check if thread is still alive
            if not reader_thread.is_alive():
                print("[ERROR] Serial reader thread has died!")
                break
            
            # Check for data from serial reader (non-blocking)
            try:
                data_lines = data_queue.get(timeout=0.1)
            except queue.Empty:
                # Process matplotlib events even when no data
                plt.pause(0.01)
                continue
            
            # Create DataFrame from collected data points
            if data_lines:
                try:
                    # data_lines is a list of lists: [[time, prop, deriv, integral, output, desiredValue, angle], ...]
                    df = pd.DataFrame(data_lines, columns=dataColumns)
                    
                    print(f"[PLOT] DataFrame created with {len(df)} rows")
                    print(f"[PLOT] Columns: {list(df.columns)}")
                    print(f"[PLOT] Data sample:\n{df.head()}")
                    
                except Exception as e:
                    print(f"[PLOT ERROR] Error creating DataFrame: {e}")
                    print(f"[PLOT ERROR] data_lines: {data_lines}")
                    continue
                    
                # Plot data from the dataframe
                try:
                    print(f"[PLOT] Plotting on subplot [0, {i}] and [1, {i}]")
                    
                    # Clear the axes before replotting
                    axs[0, i].cla()
                    axs[1, i].cla()
                    
                    # Plot desiredValue and angle (angle data)
                    try:
                        axs[0, i].plot(df[timeCol], df['desiredValue'], label='Desired Value', marker='.')
                        axs[0, i].plot(df[timeCol], df['angle'], label='Angle', marker='.')
                        axs[0, i].set_xlabel("time (s)")
                        axs[0, i].set_ylabel("Angle")
                        axs[0, i].legend(fontsize=8)
                        axs[0, i].grid(True)
                    except Exception as e:
                        print(f"[PLOT ERROR] Angle data: {e}")
                    
                    # Plot proportional, derivative, integral, and output (constant data)
                    try:
                        axs[1, i].plot(df[timeCol], df['proportional'], label='Proportional', marker='.')
                        axs[1, i].plot(df[timeCol], df['derivative'], label='Derivative', marker='.')
                        axs[1, i].plot(df[timeCol], df['integral'], label='Integral', marker='.')
                        axs[1, i].plot(df[timeCol], df['output'], label='Output', marker='.')
                        axs[1, i].set_xlabel("time (s)")
                        axs[1, i].set_ylabel("PID Components")
                        axs[1, i].legend(fontsize=8)
                        axs[1, i].grid(True)
                    except Exception as e:
                        print(f"[PLOT ERROR] Constant data: {e}")
                    
                    if i < 5:
                        i += 1
                    else:
                        i = 0
                    
                    plt.draw()
                    plt.pause(0.05)
                    
                except Exception as e:
                    print(f"[PLOT ERROR] Error plotting: {e}")
                    continue
        
        except KeyboardInterrupt:
            print("\nProgram interrupted by user")
            break
        except Exception as e:
            print(f"Unexpected error in main loop: {e}")
            continue

except KeyboardInterrupt:
    print("\nProgram interrupted by user")
except Exception as e:
    print(f"Critical error in main program: {e}")
finally:
    try:
        # Stop the serial reader thread
        stop_event.set()
        reader_thread.join(timeout=1)
        
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed")
    except Exception as e:
        print(f"Error closing serial port: {e}")
    plt.close('all')
    print("Program terminated")