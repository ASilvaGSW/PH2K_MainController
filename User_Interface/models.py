from flask_sqlalchemy import SQLAlchemy
from datetime import datetime

db = SQLAlchemy()

class User(db.Model):
    __tablename__ = 'users'
    
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(80), unique=True, nullable=False)
    password = db.Column(db.String(120), nullable=False)
    badge_id = db.Column(db.String(50), nullable=False)
    is_admin = db.Column(db.Integer)
    
class UserPreference(db.Model):
    __tablename__ = 'user_preferences'
    
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(80), nullable=False)
    language = db.Column(db.String(5), default='en')  # 'en', 'es', 'jp'

class ProdHis(db.Model):
    __tablename__ = 'prod_his'
    
    id = db.Column(db.Integer, primary_key=True)
    machine = db.Column(db.String(100), nullable=False)
    date = db.Column(db.Date, nullable=False)
    part_number = db.Column(db.String(100), nullable=False)
    qty = db.Column(db.Integer, nullable=False)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)
    user = db.Column(db.String(100), nullable=False)
    deleted = db.Column(db.Boolean, default=False)

class PartNumber(db.Model):
    __tablename__ = 'part_numbers'
    
    id = db.Column(db.Integer, primary_key=True)
    part_number = db.Column(db.String(100), unique=True, nullable=False)
    name = db.Column(db.String(200), nullable=False)
    total_length = db.Column(db.Float, nullable=True)
    created_by = db.Column(db.String(80), nullable=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    updated_at = db.Column(db.DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    active = db.Column(db.Boolean, default=True)

class Tape(db.Model):
    __tablename__ = 'tapes'
    
    id = db.Column(db.Integer, primary_key=True)
    part_number_id = db.Column(db.Integer, db.ForeignKey('part_numbers.id'), nullable=False)
    color = db.Column(db.String(50), nullable=False)
    width = db.Column(db.Float, nullable=False)
    position = db.Column(db.Float, nullable=False)
    active = db.Column(db.Boolean, default=True)

class Stamp(db.Model):
    __tablename__ = 'stamps'
    
    id = db.Column(db.Integer, primary_key=True)
    part_number_id = db.Column(db.Integer, db.ForeignKey('part_numbers.id'), nullable=False)
    color = db.Column(db.String(50), nullable=False)
    position = db.Column(db.Float, nullable=False)
    active = db.Column(db.Boolean, default=True)

class Insertion(db.Model):
    __tablename__ = 'insertions'
    
    id = db.Column(db.Integer, primary_key=True)
    part_number_id = db.Column(db.Integer, db.ForeignKey('part_numbers.id'), nullable=False)
    side = db.Column(db.String(1), nullable=False)  # 'A' or 'B'
    component = db.Column(db.String(100), nullable=False)
    active = db.Column(db.Boolean, default=True)

# Define Nozzle model for nozzle components (Side B insertions)
class Nozzle(db.Model):
    __tablename__ = 'nozzles'
    
    id = db.Column(db.Integer, primary_key=True)
    model = db.Column(db.String(100), unique=True, nullable=False)
    description = db.Column(db.Text, nullable=True)
    created_by = db.Column(db.String(80), nullable=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    updated_at = db.Column(db.DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    active = db.Column(db.Boolean, default=True)

# Define Joint model for joint components (Side A insertions)
class Joint(db.Model):
    __tablename__ = 'joints'
    
    id = db.Column(db.Integer, primary_key=True)
    model = db.Column(db.String(100), unique=True, nullable=False)
    description = db.Column(db.Text, nullable=True)
    created_by = db.Column(db.String(80), nullable=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    updated_at = db.Column(db.DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    active = db.Column(db.Boolean, default=True)

# Define Tool model for tooling management with one-to-many relationship to part numbers
class Tool(db.Model):
    __tablename__ = 'tools'
    
    id = db.Column(db.Integer, primary_key=True)
    part_number_id = db.Column(db.Integer, db.ForeignKey('part_numbers.id'), nullable=False)
    tool_number = db.Column(db.String(100), nullable=False)
    description = db.Column(db.Text, nullable=True)
    tool_type = db.Column(db.String(50), nullable=False)  # e.g., 'cutting', 'forming', 'assembly'
    quantity = db.Column(db.Integer, default=1)
    created_by = db.Column(db.String(80), nullable=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    updated_at = db.Column(db.DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    active = db.Column(db.Boolean, default=True)
    
    # Relationship to PartNumber
    part_number = db.relationship('PartNumber', backref='tools')

class DeviceType(db.Model):
    __tablename__ = 'device_types'
    
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(50), unique=True, nullable=False)
    description = db.Column(db.Text, nullable=True)
    created_by = db.Column(db.String(80), nullable=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    updated_at = db.Column(db.DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    active = db.Column(db.Boolean, default=True)

class Device(db.Model):
    __tablename__ = 'devices'
    
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(100), nullable=False)
    canbus_id = db.Column(db.String(10), unique=True, nullable=False)
    device_type_id = db.Column(db.Integer, db.ForeignKey('device_types.id'), nullable=False)
    device_type = db.relationship('DeviceType', backref='devices')
    created_by = db.Column(db.String(80), nullable=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    updated_at = db.Column(db.DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    active = db.Column(db.Boolean, default=True)

class DeviceFunction(db.Model):
    __tablename__ = 'device_functions'
    
    id = db.Column(db.Integer, primary_key=True)
    device_type_id = db.Column(db.Integer, db.ForeignKey('device_types.id'), nullable=False)
    function_id = db.Column(db.Integer, nullable=False)
    description = db.Column(db.String(200), nullable=False)
    bit0 = db.Column(db.Boolean, default=False)
    bit1 = db.Column(db.Boolean, default=False)
    bit2 = db.Column(db.Boolean, default=False)
    bit3 = db.Column(db.Boolean, default=False)
    bit4 = db.Column(db.Boolean, default=False)
    bit5 = db.Column(db.Boolean, default=False)
    bit6 = db.Column(db.Boolean, default=False)
    bit7 = db.Column(db.Boolean, default=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    active = db.Column(db.Boolean, default=True)
    device_type = db.relationship('DeviceType', backref='device_functions')

class WorkOrder(db.Model):
    __tablename__ = 'work_orders'
    
    id = db.Column(db.Integer, primary_key=True)
    part_number = db.Column(db.String(100), nullable=False)
    qty = db.Column(db.Integer, nullable=False)
    balance = db.Column(db.Integer, nullable=False)
    type = db.Column(db.String(20), nullable=False)  # 'service' or 'kanban'
    created = db.Column(db.DateTime, default=datetime.utcnow)
    started_time = db.Column(db.DateTime, nullable=True)
    end_time = db.Column(db.DateTime, nullable=True)
    user = db.Column(db.String(80), nullable=False)
    deleted = db.Column(db.Boolean, default=False)
    deleted_at = db.Column(db.DateTime, nullable=True)
    deleted_user = db.Column(db.String(80), nullable=True)

class PreventiveMaintenance(db.Model):
    __tablename__ = 'preventive_maintenance'
    
    id = db.Column(db.Integer, primary_key=True)
    device_id = db.Column(db.Integer, db.ForeignKey('devices.id'), nullable=False)
    counter = db.Column(db.Integer, default=0)  # Current counter
    lifetime = db.Column(db.Integer, nullable=False)  # Total lifetime cycles
    last_pm = db.Column(db.Date, nullable=True)  # Last preventive maintenance date
    next_pm = db.Column(db.Date, nullable=True)  # Next preventive maintenance date
    created_by = db.Column(db.String(80), nullable=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    updated_at = db.Column(db.DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    active = db.Column(db.Boolean, default=True)
    
    # Relationship
    device = db.relationship('Device', backref='preventive_maintenance')

def initialize_database(app):
    """Initialize database with sample data"""
    db.create_all()
        
        # Check if we need to add sample data
    if User.query.count() == 0:
        # Create sample users
        admin_user = User(username='admin', password='admin123')
        engineer_user = User(username='engineer', password='eng123')
        operator_user = User(username='operator', password='op123')
        
        db.session.add(admin_user)
        db.session.add(engineer_user)
        db.session.add(operator_user)
        
        # Create sample user preferences
        admin_pref = UserPreference(username='admin', language='en')
        engineer_pref = UserPreference(username='engineer', language='en')
        operator_pref = UserPreference(username='operator', language='en')
        
        db.session.add(admin_pref)
        db.session.add(engineer_pref)
        db.session.add(operator_pref)
        
        # Create sample nozzles
        nozzle1 = Nozzle(model='NZ-001', description='Standard nozzle for Side B', created_by='admin')
        nozzle2 = Nozzle(model='NZ-002', description='High pressure nozzle for Side B', created_by='admin')
        nozzle3 = Nozzle(model='NZ-003', description='Precision nozzle for Side B', created_by='admin')
        
        db.session.add(nozzle1)
        db.session.add(nozzle2)
        db.session.add(nozzle3)
        
        # Create sample joints
        joint1 = Joint(model='JT-001', description='Standard joint for Side A', created_by='admin')
        joint2 = Joint(model='JT-002', description='Heavy duty joint for Side A', created_by='admin')
        joint3 = Joint(model='JT-003', description='Flexible joint for Side A', created_by='admin')
        
        db.session.add(joint1)
        db.session.add(joint2)
        db.session.add(joint3)
        
        # Create sample device types
        device_type1 = DeviceType(name='Camera', description='Vision and imaging devices', created_by='admin')
        device_type2 = DeviceType(name='Sensor', description='Measurement and detection devices', created_by='admin')
        device_type3 = DeviceType(name='Actuator', description='Motion and control devices', created_by='admin')
        device_type4 = DeviceType(name='Controller', description='Processing and control units', created_by='admin')
        
        db.session.add(device_type1)
        db.session.add(device_type2)
        db.session.add(device_type3)
        db.session.add(device_type4)
        
        # Commit device types first to get their IDs
        db.session.commit()
        
        # Create sample devices
        device1 = Device(name='Entity #1', canbus_id='0x101', device_type_id=device_type1.id, created_by='admin')
        device2 = Device(name='Entity #2', canbus_id='0x102', device_type_id=device_type1.id, created_by='admin')
        device3 = Device(name='Entity #3', canbus_id='0x103', device_type_id=device_type2.id, created_by='admin')
        device4 = Device(name='Entity #4', canbus_id='0x104', device_type_id=device_type3.id, created_by='admin')
        
        db.session.add(device1)
        db.session.add(device2)
        db.session.add(device3)
        db.session.add(device4)
        
        # Create sample device functions
        device_func1 = DeviceFunction(
            device_type_id=device_type1.id,
            function_id=1,
            description='Camera capture function',
            bit0=True,
            bit1=False,
            bit2=True
        )
        
        device_func2 = DeviceFunction(
            device_type_id=device_type1.id,
            function_id=2,
            description='Camera zoom function',
            bit0=False,
            bit1=True,
            bit3=True
        )
        
        device_func3 = DeviceFunction(
            device_type_id=device_type2.id,
            function_id=1,
            description='Sensor reading function',
            bit0=True,
            bit2=True,
            bit4=True
        )
        
        device_func4 = DeviceFunction(
            device_type_id=device_type3.id,
            function_id=1,
            description='Actuator movement function',
            bit1=True,
            bit3=True,
            bit5=True
        )
        
        db.session.add(device_func1)
        db.session.add(device_func2)
        db.session.add(device_func3)
        db.session.add(device_func4)
        db.session.add(device3)
        db.session.add(device4)
        
        # Create sample part numbers
        part1 = PartNumber(part_number='PH2K-001', name='Standard Power Hose Assembly', total_length=150.0, created_by='admin')
        part2 = PartNumber(part_number='PH2K-002', name='Heavy Duty Power Hose Assembly', total_length=200.0, created_by='admin')
        part3 = PartNumber(part_number='PH2K-003', name='Compact Power Hose Assembly', total_length=100.0, created_by='admin')
        part4 = PartNumber(part_number='PH2K-004', name='Extended Power Hose Assembly', total_length=300.0, created_by='admin')
        part5 = PartNumber(part_number='PH2K-005', name='High Pressure Power Hose Assembly', total_length=180.0, created_by='admin')
        
        db.session.add(part1)
        db.session.add(part2)
        db.session.add(part3)
        db.session.add(part4)
        db.session.add(part5)
        
        # Commit part numbers first to get their IDs
        db.session.commit()
        
        # Create sample preventive maintenance data
        from datetime import date, timedelta
        
        pm1 = PreventiveMaintenance(
            device_id=device1.id,
            counter=100,
            lifetime=150,
            last_pm=date(2025, 1, 21),
            next_pm=date(2025, 1, 21),
            created_by='admin'
        )
        
        pm2 = PreventiveMaintenance(
            device_id=device2.id,
            counter=100,
            lifetime=150,
            last_pm=date(2025, 1, 21),
            next_pm=date(2025, 1, 21),
            created_by='admin'
        )
        
        pm3 = PreventiveMaintenance(
            device_id=device3.id,
            counter=100,
            lifetime=160,
            last_pm=date(2025, 1, 21),
            next_pm=date(2025, 1, 21),
            created_by='admin'
        )
        
        pm4 = PreventiveMaintenance(
            device_id=device4.id,
            counter=75,
            lifetime=120,
            last_pm=date(2025, 1, 15),
            next_pm=date(2025, 2, 20),
            created_by='admin'
        )
        
        db.session.add(pm1)
        db.session.add(pm2)
        db.session.add(pm3)
        db.session.add(pm4)
        
        # Create sample work orders
        
        work_order1 = WorkOrder(
            part_number='PH2K-001',
            qty=100,
            balance=75,
            type='kanban',
            user='operator',
            started_time=datetime.utcnow() - timedelta(hours=2)
        )
        
        work_order2 = WorkOrder(
            part_number='PH2K-002',
            qty=50,
            balance=50,
            type='service',
            user='engineer'
        )
        
        work_order3 = WorkOrder(
            part_number='PH2K-003',
            qty=200,
            balance=150,
            type='kanban',
            user='operator',
            started_time=datetime.utcnow() - timedelta(hours=4),
            end_time=datetime.utcnow() - timedelta(hours=1)
        )
        
        work_order4 = WorkOrder(
            part_number='PH2K-004',
            qty=25,
            balance=25,
            type='service',
            user='admin'
        )
        
        work_order5 = WorkOrder(
            part_number='PH2K-005',
            qty=75,
            balance=60,
            type='kanban',
            user='operator',
            started_time=datetime.utcnow() - timedelta(minutes=30)
        )
        
        # Add a deleted work order example
        work_order6 = WorkOrder(
            part_number='PH2K-001',
            qty=30,
            balance=0,
            type='service',
            user='engineer',
            deleted=True,
            deleted_at=datetime.utcnow() - timedelta(days=1),
            deleted_user='admin'
        )
        
        db.session.add(work_order1)
        db.session.add(work_order2)
        db.session.add(work_order3)
        db.session.add(work_order4)
        db.session.add(work_order5)
        db.session.add(work_order6)
        
        # Create sample production history data
        from datetime import timedelta
        
        # Generate production data for the last 30 days
        base_date = datetime.now().date() - timedelta(days=30)
        
        for i in range(30):
            current_date = base_date + timedelta(days=i)
            
            # Create PNA (Pass No Action) records
            pna_record1 = ProdHis(
                machine='PH2K-LINE-01',
                date=current_date,
                part_number='PH2K-001',
                qty=45 + (i % 15),  # Varying quantities
                timestamp=datetime.combine(current_date, datetime.min.time()) + timedelta(hours=9, minutes=30),
                user='operator',
                deleted=False
            )
            
            pna_record2 = ProdHis(
                machine='PH2K-LINE-01',
                date=current_date,
                part_number='PH2K-002',
                qty=35 + (i % 20),
                timestamp=datetime.combine(current_date, datetime.min.time()) + timedelta(hours=11, minutes=15),
                user='operator',
                deleted=False
            )
            
            # Create PMA (Pass Manual Action) records
            pma_record1 = ProdHis(
                machine='PH2K-LINE-02',
                date=current_date,
                part_number='PH2K-003',
                qty=25 + (i % 10),
                timestamp=datetime.combine(current_date, datetime.min.time()) + timedelta(hours=13, minutes=45),
                user='engineer',
                deleted=False
            )
            
            pma_record2 = ProdHis(
                machine='PH2K-LINE-02',
                date=current_date,
                part_number='PH2K-004',
                qty=30 + (i % 12),
                timestamp=datetime.combine(current_date, datetime.min.time()) + timedelta(hours=15, minutes=20),
                user='engineer',
                deleted=False
            )
            
            # Add additional records for variety
            if i % 3 == 0:  # Every third day
                extra_record = ProdHis(
                    machine='PH2K-LINE-01',
                    date=current_date,
                    part_number='PH2K-005',
                    qty=20 + (i % 8),
                    timestamp=datetime.combine(current_date, datetime.min.time()) + timedelta(hours=16, minutes=30),
                    user='operator',
                    deleted=False
                )
                db.session.add(extra_record)
            
            db.session.add(pna_record1)
            db.session.add(pna_record2)
            db.session.add(pma_record1)
            db.session.add(pma_record2)
        
        db.session.commit()
        print("Database initialized with sample data")