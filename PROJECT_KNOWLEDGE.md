# POWER HOSE 2K - Project Knowledge Base

## Project Overview
POWER HOSE 2K is an industrial automation system with a Flask-based web interface for controlling and monitoring hose manufacturing processes. The system includes multiple hardware components controlled via CAN bus communication.

## Project Structure

### Main Directories
- **User_Interface/**: Flask web application
- **Firmware/**: Arduino firmware for various hardware components
- **main_controller/**: Python classes for hardware control
- **PNG/**: UI design mockups and screenshots

### Key Files
- `User_Interface/app.py`: Main Flask application
- `User_Interface/routes/testing_routes.py`: Testing functionality routes
- `User_Interface/templates/testing.html`: Testing dashboard UI
- `User_Interface/models.py`: Database models
- `User_Interface/run_server.py`: Server startup script

## Recent Changes and Fixes

### Testing Dashboard (testing.html)
1. **Button Layout Optimization**: All 8 test action buttons now fit in a single row
   - Changed grid from `col-lg-3 col-md-4 col-sm-6` to `col`
   - Reduced button height to `100px`
   - Decreased font size to `0.85rem`
   - Reduced grid gap to `g-2`

2. **Button Labels Updated**:
   - Test Action 4: "Pick and Place" (English), "Recoger y Colocar" (Spanish), "ピックアンドプレース" (Japanese)
   - Test Action 5: "Insertion" (English), "Inserción" (Spanish), "挿入" (Japanese)
   - Test Action 6: "Hose Pulling" (English), "Tirar Manguera" (Spanish), "ホース引き" (Japanese)
   - Test Action 7: "Full Routine" (English), "Rutina Completa" (Spanish), "フルルーチン" (Japanese)
   - Test Action 8: "Insertion Full Cycle" (English), "Ciclo Completo de Inserción" (Spanish), "挿入フルサイクル" (Japanese)

3. **Icons Updated**:
   - Pick and Place: `bi-box-arrow-in-down`
   - Insertion: `bi-arrow-down-circle`
   - Hose Pulling: `bi-arrow-left-right`
   - Full Routine: `bi-list-check`

4. **Test Results Section**:
   - Fixed height of 300px with scrollbar
   - Newest comments appear at the top
   - Auto-scroll to top for new entries

5. **Button State Management**:
   - All buttons disabled during any test execution
   - Active button shows spinning icon and "Processing..." text
   - Buttons re-enabled after backend response

### Backend Fixes
1. **Route Conflict Resolution** (app.py):
   - Changed `details_management` route from `/device_management` to `/details_management`
   - Now both `/details_management` and `/device_management` routes are distinct

2. **Function Call Fix** (testing_routes.py):
   - Fixed `test_action_4`: Changed `movePickandPlace` to `movePickandPlace()` (added parentheses)
   - This resolved the issue where test_action_4 was not working

## Multi-Language Support
The application supports three languages:
- **English** (default)
- **Spanish** (Español)
- **Japanese** (日本語)

All UI elements use the `lang-text` class with data attributes:
```html
<span class="lang-text" data-en="English" data-es="Español" data-jp="日本語">English</span>
```

## Hardware Components (Firmware)
- **elevator_in**: Elevator input mechanism
- **elevator_out**: Elevator output mechanism
- **hose_jig**: Hose positioning jig
- **hose_puller**: Hose pulling mechanism
- **insertion_jig**: Insertion positioning
- **insertion_servos**: Servo-controlled insertion
- **pick_and_place**: Pick and place mechanism
- **puller_extension**: Puller extension mechanism

## Python Hardware Classes (main_controller/classes/)
- `canbus.py`: CAN bus communication
- `canbus_jetson.py`: Jetson-specific CAN bus
- `elevator_in.py`: Elevator input control
- `hose_jig.py`: Hose jig control
- `hose_puller.py`: Hose puller control
- `insertion_jig.py`: Insertion jig control
- `insertion_servos.py`: Servo control
- `pick_and_place.py`: Pick and place control
- `puller_extension.py`: Puller extension control

## Testing Functions
1. **Check Canbus**: Validates CAN bus communication
2. **Validate MCU**: Checks microcontroller units
3. **Feed Material**: Material feeding operation
4. **Pick and Place**: Pick and place operation
5. **Insertion**: Insertion operation
6. **Hose Pulling**: Hose pulling operation
7. **Full Routine**: Complete manufacturing cycle
8. **Insertion Full Cycle**: Complete insertion cycle

## Development Guidelines
- Maintain consistent styling across all templates
- Always include all three language translations
- Use Bootstrap 5.3.3 and Bootstrap Icons
- Follow responsive design principles
- Update `models.py` when creating new database tables
- No cancel buttons in modals (use X or click outside)
- Git operations don't require approval

## Database
- SQLite database managed through Flask-SQLAlchemy
- Models defined in `User_Interface/models.py`
- Database initialization via `create_db.py`

## Current Status
- All testing dashboard improvements implemented
- Route conflicts resolved
- Function call issues fixed
- Multi-language support fully functional
- Responsive design optimized for all screen sizes

## Next Steps
The system is ready for continued development. Key areas for potential enhancement:
- Additional test functions
- Real-time monitoring dashboards
- Production reporting features
- Preventive maintenance scheduling
- Device management improvements

---
*Last Updated: Based on conversation history through testing dashboard optimizations and backend fixes*