from Classes.AICamera import AICamera
import cv2
import platform
import time

if platform.system() == "Windows":
    from Classes.canbus import Canbus
else:
    from Classes.canbus_jetson import Canbus

from Classes.pick_and_place import PickAndPlace

canbus = Canbus()
pick_and_place = PickAndPlace(canbus, 0x191)

canbus_id = 0x001
reply_id = 0x401

def initialize_canbus():
    
    if canbus.start_canbus():
        print("Canbus started successfully")
    else:
        print("Failed to start Canbus")

# Inicializar cámaras
# Valores de pick_up actualizados para coincidir con webtester_real.py (Nozzle=94, Joint=80)
# Configuración de cámaras: Nozzle=0, Joint=2 (Joint probado en webtester con índice 2)
cameraN = AICamera(2, pick_up=91)
cameraJ = AICamera(0, 'joint', pick_up=80, total=50, positions=10)

def debug():

    # Cargar Modelo
    cameraN.load_model(0)
    cameraJ.load_model(1)
    
    # Capturar frames de ambas cámaras
    frameN = cameraN.capture_frame()
    frameJ = cameraJ.capture_frame()

    # Contar objetos en cada frame
    countN = cameraN.count_objects()
    countJ = cameraJ.count_objects()
    
    print(f"Objects in Nozzle Camera: {countN}")
    print(f"Objects in Joint Camera: {countJ}")

    # Obtener y mostrar matrices de detección
    matrixN = cameraN.get_detection_matrix()
    matrixJ = cameraJ.get_detection_matrix()
    
    print("\nMatrix Nozzle:")
    for row in matrixN:
        print(row)
        
    print("\nMatrix Joint:")
    for row in matrixJ:
        print(row)

    # Closest Distance for Alignment 
    closest_obj_N = cameraN.get_y_distance_to_closest()
    closest_obj_J = cameraJ.get_y_distance_to_closest()
    print(f"Closest object in Nozzle Camera: {closest_obj_N}")
    print(f"Closest object in Joint Camera: {closest_obj_J}")

    # Closest Row for Pick Up
    closes_pickup = cameraN.get_closest_row_matrix()
    closes_joint = cameraJ.get_closest_row_matrix()
    print(f"Closest pickup in Nozzle Camera: {closes_pickup}")
    print(f"Closest joint in Joint Camera: {closes_joint}")

def process_can_instruction(message):
    if message.data:
        b0 = message.data[0]
        print(f"Instruction received (first byte): {b0}")

        # Reset not needed
        if b0 == 0x01:
            pass
        
        # Heartbit
        elif b0 == 0x02:
            canbus.send_message(reply_id, [0x02])

        # Load Model Nozzle
        elif b0 == 0x03:
            cameraN.load_model(0)
            if cameraN.model:
                canbus.send_message(reply_id, [b0, 0x01]) # Success
            else:
                canbus.send_message(reply_id, [b0, 0x00]) # Fail

        # Load Model Joint
        elif b0 == 0x04:
            cameraJ.load_model(1)
            if cameraJ.model:
                canbus.send_message(reply_id, [b0, 0x01]) # Success
            else:
                canbus.send_message(reply_id, [b0, 0x00]) # Fail

        # Count Objects Nozzle
        elif b0 == 0x05:
            if cameraN.model:
                count = cameraN.count_objects()
                print("Countend Nozzles : " , count)
                canbus.send_message(reply_id, [b0, count & 0xFF])
            else:
                canbus.send_message(reply_id, [b0, 0xFF]) # Error code

        # Count Objects Joint
        elif b0 == 0x06:
            if cameraJ.model:
                count = cameraJ.count_objects()
                print("Countend Joints: ", count)
                canbus.send_message(reply_id, [b0, count & 0xFF])
            else:
                canbus.send_message(reply_id, [b0, 0xFF]) # Error code

        # Alignment Nozzle (Left Conveyor)
        elif b0 == 0x07:
            if cameraN.model:

                # Flush de frames para asegurar lectura actual
                for _ in range(5):
                    cameraN.capture_frame()

                print("Starting alignment for Nozzle (Left Conveyor)...")
                conveyor_active = True
                pick_and_place.start_left_conveyor(100) # Iniciar conveyor izquierdo
                
                final_distance = None
                           
                # Bucle de alineación
                while True:
                    # cameraN.capture_frame() # Removed redundant call to avoid buffer buildup
                    distance = cameraN.get_y_distance_to_closest()
                    
                    if distance is not None:
                        print(f"Distance: {distance:.2f}")
                        if abs(distance) <= 6.5:
                            print(f"Target reached. Distance: {distance:.2f}. Stopping conveyor.")
                            pick_and_place.stop_left_conveyor()
                            conveyor_active = False
                            final_distance = distance
                            break
                    else:
                        # Si se pierde el objeto, podríamos decidir parar o seguir
                        pass
                    
                    # Pequeña pausa para no saturar CPU y permitir que la cámara actualice
                    # time.sleep(0.01) 
                
                if final_distance is not None:
                    dist_int = int(final_distance)
                    data = [b0] + list(dist_int.to_bytes(2, 'big', signed=True)) # signed=True para distancias negativas
                    canbus.send_message(reply_id, data)
                else:
                    canbus.send_message(reply_id, [b0, 0xFF, 0xFF])
            else:
                canbus.send_message(reply_id, [b0, 0xFF, 0xFE]) # Error code for model not loaded

        # Alignment Joint (Right Conveyor)
        elif b0 == 0x08:
            if cameraJ.model:

                     # Flush de frames para asegurar lectura actual
                for _ in range(5):
                    cameraJ.capture_frame()


                print("Starting alignment for Joint (Right Conveyor)...")
                conveyor_active = True
                pick_and_place.start_right_conveyor(50) # Iniciar conveyor derecho
                
                final_distance = None
                
                # Bucle de alineación
                while True:
                    # cameraJ.capture_frame() # Removed redundant call
                    distance = cameraJ.get_y_distance_to_closest()
                    
                    if distance is not None:
                        print(f"Distance: {distance:.2f}")
                        if abs(distance) <= 7.5:
                            print(f"Target reached. Distance: {distance:.2f}. Stopping conveyor.")
                            pick_and_place.stop_right_conveyor()
                            conveyor_active = False
                            final_distance = distance
                            break
                    else:
                        pass
                    
                    # Pequeña pausa para no saturar CPU
                    time.sleep(0.01)

                if final_distance is not None:
                    dist_int = int(final_distance)
                    data = [b0] + list(dist_int.to_bytes(2, 'big', signed=True))
                    canbus.send_message(reply_id, data)
                else:
                    canbus.send_message(reply_id, [b0, 0xFF, 0xFF])
            else:
                canbus.send_message(reply_id, [b0, 0xFF, 0xFE]) # Error code for model not loaded

        # Pick Up Nozzle
        elif b0 == 0x09:
            if cameraN.model:
                matrix = cameraN.get_closest_row_matrix()
                print(matrix)
                if matrix:
                    try:
                        first_one_index = matrix.index(1)
                        print(f"CAN 0x09: First '1' is at position: {first_one_index}")
                    except ValueError:
                        print("CAN 0x09: No '1' found in the matrix.")
                    byte_val = 0
                    for i, bit in enumerate(matrix):
                        if bit == 1:
                            byte_val |= (1 << i)
                    canbus.send_message(reply_id, [b0, byte_val])
                else:
                    canbus.send_message(reply_id, [b0, 0x00]) # Empty matrix
            else:
                canbus.send_message(reply_id, [b0, 0xFF]) # Error code

        # Pick Up Joint
        elif b0 == 0x10:
            if cameraJ.model:
                matrix = cameraJ.get_closest_row_matrix()
                print(matrix)
                if matrix:
                    try:
                        first_one_index = matrix.index(1)
                        print(f"CAN 0x10: First '1' is at position: {first_one_index}")
                    except ValueError:
                        print("CAN 0x10: No '1' found in the matrix.")
                    mask = 0
                    for i, bit in enumerate(matrix):
                        if bit == 1:
                            mask |= (1 << i)
                    data = [b0] + list(mask.to_bytes(2, 'big'))
                    canbus.send_message(reply_id, data)
                else:
                    canbus.send_message(reply_id, [b0, 0x00, 0x00]) # Empty matrix
            else:
                canbus.send_message(reply_id, [b0, 0xFF, 0xFF]) # Error code

        # Inspection Camera 0
        elif b0 == 0x11:
            if cameraN.model:
                good = cameraN.check_for_good_class()
                canbus.send_message(reply_id, [b0, 0x01 if good else 0x00])
            else:
                canbus.send_message(reply_id, [b0, 0xFF]) # Error code
                
        # Inspection Camera 1
        elif b0 == 0x12:
            if cameraJ.model:
                good = cameraJ.check_for_good_class()
                canbus.send_message(reply_id, [b0, 0x01 if good else 0x00])
            else:
                canbus.send_message(reply_id, [b0, 0xFF]) # Error code

        # Show Frame Nozzle
        elif b0 == 0x13:
            # Usar get_annotated_frame si el modelo está cargado, sino capture_frame normal
            if cameraN.model:
                 frame = cameraN.get_annotated_frame()
            else:
                 frame = cameraN.capture_frame()

            if frame is not None:
                cv2.imshow("Nozzle Camera", frame)
                # Esperar un poco más para asegurar que la ventana se dibuje correctamente
                # Un bucle de waitKey ayuda a procesar los eventos de la GUI
                for _ in range(10):
                    cv2.waitKey(10)
                canbus.send_message(reply_id, [b0, 0x01])
            else:
                canbus.send_message(reply_id, [b0, 0x00])

        # Show Frame Joint
        elif b0 == 0x14:
            if cameraJ.model:
                 frame = cameraJ.get_annotated_frame()
            else:
                 frame = cameraJ.capture_frame()
                 
            if frame is not None:
                cv2.imshow("Joint Camera", frame)
                # Esperar un poco más para asegurar que la ventana se dibuje correctamente
                for _ in range(10):
                    cv2.waitKey(10)
                canbus.send_message(reply_id, [b0, 0x01])
            else:
                canbus.send_message(reply_id, [b0, 0x00])

        else:
            print(f"Unknown instruction: {b0}")
    else:
        print("Received CAN message with no data.")

    

class MockMessage:
    def __init__(self, data):
        self.data = data

def main():

    # debug()

    # while True:
    #     message = canbus.read_message()
    #     if message is not None and message.arbitration_id == canbus_id:
    #         process_can_instruction(message)


    # # Load models and capture initial frames
    # print("Loading models...")
    # cameraN.load_model(0)
    # cameraJ.load_model(1)
    # print("Models loaded.")
    #
    # print("Capturing frames...")
    # cameraN.capture_frame()
    # cameraJ.capture_frame()
    # print("Frames captured.")

    while True:
        print("\n--- CAN Instruction Test Menu ---")
        print("1. Heartbeat (0x02)")
        print("2. Load Model Nozzle (0x03)")
        print("3. Load Model Joint (0x04)")
        print("4. Count Objects Nozzle (0x05)")
        print("5. Count Objects Joint (0x06)")
        print("6. Alignment Nozzle (0x07)")
        print("7. Alignment Joint (0x08)")
        print("8. Pick Up Nozzle (0x09)")
        print("9. Pick Up Joint (0x10)")
        print("10. Inspection Camera 0 (0x11)")
        print("11. Inspection Camera 1 (0x12)")
        print("12. Show Frame Nozzle (0x13)")
        print("13. Show Frame Joint (0x14)")
        print("99. Rerun frame capture")
        print("0. Exit")

        choice = input("Enter option: ")

        instruction = -1
        if choice == '0':
            break
        elif choice == '1':
            instruction = 0x02
        elif choice == '2':
            instruction = 0x03
        elif choice == '3':
            instruction = 0x04
        elif choice == '4':
            instruction = 0x05
        elif choice == '5':
            instruction = 0x06
        elif choice == '6':
            instruction = 0x07
        elif choice == '7':
            instruction = 0x08
        elif choice == '8':
            instruction = 0x09
        elif choice == '9':
            instruction = 0x10
        elif choice == '10':
            instruction = 0x11
        elif choice == '11':
            instruction = 0x12
        elif choice == '12':
            instruction = 0x13
        elif choice == '13':
            instruction = 0x14
        elif choice == '99':
            print("Capturing frames...")
            cameraN.capture_frame()
            cameraJ.capture_frame()
            print("Frames captured.")
            continue
        else:
            print("Invalid choice.")
            continue

        try:
            message = MockMessage([instruction])
            process_can_instruction(message)
        except Exception as e:
            print(f"An error occurred: {e}")


if __name__ == "__main__":

    initialize_canbus()
    main()
