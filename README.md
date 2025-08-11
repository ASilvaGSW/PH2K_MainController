# PH2K MainController

## English

The PH2K MainController is a comprehensive system for controlling and monitoring the PH2K production line. This project includes firmware for various components, a user interface for operation and management, and a main controller for coordinating all subsystems.

### Project Structure

- **Firmware**: Contains Arduino code for various hardware components:
  - Elevator In/Out
  - Hose Jig
  - Hose Puller
  - Insertion Jig
  - Insertion Servos
  - Pick and Place
  - Puller Extension

- **User Interface**: Flask-based web application for:
  - Process creation and management
  - Production reporting
  - System setup and configuration
  - Engineering tools

- **Main Controller**: Python-based central control system that coordinates all components via CAN bus communication

### Requirements

- Python 3.8+
- MySQL Database
- Arduino IDE (for firmware development)
- Flask and related packages (see requirements.txt)

### Setup and Installation

1. Set up the MySQL database
2. Install Python dependencies: `pip install -r User_Interface/requirements.txt`
3. Configure database connection in `User_Interface/app.py`
4. Run the application: `python User_Interface/run_server.py`

## Español

El PH2K MainController es un sistema integral para controlar y monitorear la línea de producción PH2K. Este proyecto incluye firmware para varios componentes, una interfaz de usuario para operación y gestión, y un controlador principal para coordinar todos los subsistemas.

### Estructura del Proyecto

- **Firmware**: Contiene código Arduino para varios componentes de hardware:
  - Elevador de Entrada/Salida
  - Plantilla de Manguera
  - Extractor de Manguera
  - Plantilla de Inserción
  - Servos de Inserción
  - Pick and Place
  - Extensión de Extractor

- **Interfaz de Usuario**: Aplicación web basada en Flask para:
  - Creación y gestión de procesos
  - Informes de producción
  - Configuración del sistema
  - Herramientas de ingeniería

- **Controlador Principal**: Sistema de control central basado en Python que coordina todos los componentes mediante comunicación CAN bus

### Requisitos

- Python 3.8+
- Base de datos MySQL
- Arduino IDE (para desarrollo de firmware)
- Flask y paquetes relacionados (ver requirements.txt)

### Configuración e Instalación

1. Configurar la base de datos MySQL
2. Instalar dependencias de Python: `pip install -r User_Interface/requirements.txt`
3. Configurar la conexión a la base de datos en `User_Interface/app.py`
4. Ejecutar la aplicación: `python User_Interface/run_server.py`

## 日本語

PH2K MainControllerは、PH2K生産ラインを制御および監視するための包括的なシステムです。このプロジェクトには、さまざまなコンポーネント用のファームウェア、操作および管理用のユーザーインターフェース、およびすべてのサブシステムを調整するメインコントローラーが含まれています。

### プロジェクト構造

- **ファームウェア**: さまざまなハードウェアコンポーネント用のArduinoコードを含みます：
  - 入出力エレベーター
  - ホースジグ
  - ホースプーラー
  - 挿入ジグ
  - 挿入サーボ
  - ピック＆プレース
  - プーラー拡張

- **ユーザーインターフェース**: Flaskベースのウェブアプリケーション：
  - プロセスの作成と管理
  - 生産レポート
  - システムセットアップと設定
  - エンジニアリングツール

- **メインコントローラー**: CANバス通信を介してすべてのコンポーネントを調整するPythonベースの中央制御システム

### 要件

- Python 3.8+
- MySQLデータベース
- Arduino IDE（ファームウェア開発用）
- Flaskおよび関連パッケージ（requirements.txtを参照）

### セットアップとインストール

1. MySQLデータベースをセットアップする
2. Pythonの依存関係をインストールする：`pip install -r User_Interface/requirements.txt`
3. `User_Interface/app.py`でデータベース接続を設定する
4. アプリケーションを実行する：`python User_Interface/run_server.py`