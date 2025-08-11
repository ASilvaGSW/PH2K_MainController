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
    deleted = db.Column(db.Integer, default=datetime.utcnow)

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
        
        db.session.commit()
        print("Database initialized with sample data")