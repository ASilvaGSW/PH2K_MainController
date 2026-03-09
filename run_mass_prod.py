"""
Script para ejecutar el panel de control de Mass Prod.
Ejecutar desde la raíz del proyecto: python run_mass_prod.py
O doble clic si Python está asociado a .py
"""
import os
import sys

# Asegurar que estamos en el directorio del proyecto
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
sys.path.insert(0, script_dir)

if __name__ == "__main__":
    try:
        from User_Interface.routes.mass_prod import init
        print("Iniciando Mass Prod (panel de control y hilos)...")
        init()
        input("Presiona Enter para salir...\n")
    except Exception as e:
        print(f"Error al iniciar: {e}")
        import traceback
        traceback.print_exc()
        input("Presiona Enter para cerrar...")
