from flask import Flask, render_template, request, redirect, url_for, session, jsonify, flash
from datetime import datetime, date, timedelta
import os

# Import database models
from models import db, User, UserPreference, ProdHis, initialize_database,PartNumber,Tape,Stamp,Insertion,Nozzle,Joint,Tool,Device,DeviceType,DeviceFunction,WorkOrder,PreventiveMaintenance,PreventiveMaintenanceComments

# Import blueprints
from routes.testing_routes import testing_bp

app = Flask(__name__)
app.config['SECRET_KEY'] = 'gswrnd2025'
app.config['SQLALCHEMY_DATABASE_URI'] = 'mysql+pymysql://root:root@localhost/maincontroller'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

# Initialize database with app
db.init_app(app)

# Initialize the database when the app starts
with app.app_context():
    initialize_database(app)

# Register blueprints
app.register_blueprint(testing_bp)

def process_badge_id(raw_badge_id):
    """
    Process badge ID from scanner format (A000010395A) to database format (10395)
    Removes 'A' characters and leading zeros
    """
    if not raw_badge_id:
        return raw_badge_id
    
    # Remove 'A' characters from beginning and end
    processed = raw_badge_id.strip('A')
    
    # Remove leading zeros
    processed = processed.lstrip('0')
    
    # If all digits were zeros, return '0'
    if not processed:
        processed = '0'
    
    return processed

@app.route('/', methods=['GET', 'POST'])
def welcome():
    if request.method == 'POST':
        username = request.form.get('userId')
        password = request.form.get('password')
        badge_id = request.form.get('badgeId')
        
        # Check if user exists based on username/password or badge ID
        user = None
        if badge_id:
            # Process badge ID to match database format
            processed_badge_id = process_badge_id(badge_id)
            user = User.query.filter_by(badge_id=processed_badge_id).first()
        else:
            user = User.query.filter_by(username=username, password=password).first()
        
        if user:
            # Store user info in session
            session['logged_in'] = True
            session['username'] = user.username
            session['is_admin'] = user.is_admin
            session['is_operator'] = user.is_operator
            
            return redirect(url_for('main'))
        else:
            # Flash error message
            flash('Invalid credentials. Please try again.', 'error')
            
    return render_template('welcome.html')

@app.route('/main')
def main():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    is_operator = session.get('is_operator', False)
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    return render_template('main.html', username=username, language=language, is_operator=is_operator)

@app.route('/set_language', methods=['POST'])
def set_language():
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    data = request.get_json()
    language = data.get('language')
    username = session.get('username')
    
    if not language or not username:
        return jsonify(success=False, message='Missing data'), 400
    
    # Find existing preference or create new one
    user_pref = UserPreference.query.filter_by(username=username).first()
    
    if user_pref:
        user_pref.language = language
    else:
        user_pref = UserPreference(username=username, language=language)
        db.session.add(user_pref)
    
    db.session.commit()
    return jsonify(success=True)

@app.route('/logout')
def logout():
    session.clear()
    return redirect(url_for('welcome'))

@app.route('/operator_screen')
def operator_screen():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    # Get active work orders
    work_orders = WorkOrder.query.filter_by(deleted=False).all()
    
    # Get part numbers for the dropdown
    part_numbers = PartNumber.query.filter_by(active=True).all()
    
    return render_template('operator_screen.html', 
                         username=username, 
                         language=language,
                         work_orders=work_orders,
                         part_numbers=part_numbers)

@app.route('/live_view')
def live_view():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    return render_template('live_view.html', 
                         username=username, 
                         language=language)

@app.route('/engineering')
def engineering():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    return render_template('engineering.html', username=username, language=language)

@app.route('/system_setup')
def system_setup():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    is_admin = session.get('is_admin', False)
    
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    # Get all users for the admin panel
    users = []
    if is_admin:
        users = User.query.all()
    
    return render_template('system_setup.html', username=username, language=language, is_admin=is_admin, users=users)

@app.route('/testing')
def testing():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    return render_template('testing.html', username=username, language=language)

@app.route('/production_report')
def production_report():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    # Get production history data
    
    # Get data for the current week
    today = datetime.now().date()
    start_of_week = today - timedelta(days=today.weekday())
    end_of_week = start_of_week + timedelta(days=6)
    
    production_data = ProdHis.query.filter(
        ProdHis.date >= start_of_week,
        ProdHis.date <= end_of_week,
        ProdHis.deleted == False
    ).order_by(ProdHis.date, ProdHis.timestamp).all()
    
    # Get unique part numbers for dropdown
    part_numbers = ProdHis.query.with_entities(ProdHis.part_number).distinct().filter_by(deleted=False).all()
    part_numbers = [p[0] for p in part_numbers]
    
    return render_template('production_report.html', 
                           username=username, 
                           language=language, 
                           production_data=production_data,
                           part_numbers=part_numbers,
                           start_date=start_of_week,
                           end_date=end_of_week,
                           datetime=datetime,
                           timedelta=timedelta)

@app.route('/add_user', methods=['POST'])
def add_user():
    # Check if user is logged in and is admin
    if not session.get('logged_in') or not session.get('is_admin'):
        return redirect(url_for('welcome'))
    
    # Get form data
    username = request.form.get('username')
    password = request.form.get('password')
    badge_id = request.form.get('badge_id')
    is_admin = 'is_admin' in request.form
    is_operator = 'is_operator' in request.form
    
    # Validate data
    if not username or not password or not badge_id:
        flash('All fields are required', 'error')
        return redirect(url_for('system_setup'))
    
    # Check if username or badge_id already exists
    existing_user = User.query.filter((User.username == username) | (User.badge_id == badge_id)).first()
    if existing_user:
        flash('Username or Badge ID already exists', 'error')
        return redirect(url_for('system_setup'))
    
    # Create new user
    new_user = User(username=username, password=password, badge_id=badge_id, is_admin=is_admin, is_operator=is_operator)
    db.session.add(new_user)
    db.session.commit()
    
    flash('User added successfully', 'success')
    return redirect(url_for('system_setup'))

@app.route('/edit_user', methods=['POST'])
def edit_user():
    # Check if user is logged in and is admin
    if not session.get('logged_in') or not session.get('is_admin'):
        return redirect(url_for('welcome'))
    
    # Get form data
    user_id = request.form.get('user_id')
    username = request.form.get('username')
    badge_id = request.form.get('badge_id')
    is_admin = 'is_admin' in request.form
    is_operator = 'is_operator' in request.form
    
    # Validate data
    if not user_id or not username or not badge_id:
        flash('All fields are required', 'error')
        return redirect(url_for('system_setup'))
    
    # Find user
    user = User.query.get(user_id)
    if not user:
        flash('User not found', 'error')
        return redirect(url_for('system_setup'))
    
    # Check if username or badge_id already exists for another user
    existing_user = User.query.filter(
        ((User.username == username) | (User.badge_id == badge_id)) & 
        (User.id != int(user_id))
    ).first()
    if existing_user:
        flash('Username or Badge ID already exists for another user', 'error')
        return redirect(url_for('system_setup'))
    
    # Update user
    user.username = username
    user.badge_id = badge_id
    user.is_admin = is_admin
    user.is_operator = is_operator
    db.session.commit()
    
    # Update user preferences if username changed
    if user.username != username:
        user_pref = UserPreference.query.filter_by(username=user.username).first()
        if user_pref:
            user_pref.username = username
            db.session.commit()
    
    flash('User updated successfully', 'success')
    return redirect(url_for('system_setup'))

@app.route('/delete_user', methods=['POST'])
def delete_user():
    # Check if user is logged in and is admin
    if not session.get('logged_in') or not session.get('is_admin'):
        return redirect(url_for('welcome'))
    
    # Get form data
    user_id = request.form.get('user_id')
    
    # Validate data
    if not user_id:
        flash('User ID is required', 'error')
        return redirect(url_for('system_setup'))
    
    # Find user
    user = User.query.get(user_id)
    if not user:
        flash('User not found', 'error')
        return redirect(url_for('system_setup'))
    
    # Prevent deleting the current user
    if user.username == session.get('username'):
        flash('Cannot delete your own account', 'error')
        return redirect(url_for('system_setup'))
    
    # Delete user preferences
    user_pref = UserPreference.query.filter_by(username=user.username).first()
    if user_pref:
        db.session.delete(user_pref)
    
    # Delete user
    db.session.delete(user)
    db.session.commit()
    
    flash('User deleted successfully', 'success')
    return redirect(url_for('system_setup'))

@app.route('/reset_password', methods=['POST'])
def reset_password():
    # Check if user is logged in and is admin
    if not session.get('logged_in') or not session.get('is_admin'):
        return redirect(url_for('welcome'))
    
    # Get form data
    user_id = request.form.get('user_id')
    password = request.form.get('password')
    
    # Validate data
    if not user_id or not password:
        flash('All fields are required', 'error')
        return redirect(url_for('system_setup'))
    
    # Find user
    user = User.query.get(user_id)
    if not user:
        flash('User not found', 'error')
        return redirect(url_for('system_setup'))
    
    # Update password
    user.password = password
    db.session.commit()
    
    flash('Password reset successfully', 'success')
    return redirect(url_for('system_setup'))

@app.route('/backup_database')
def backup_database():
    # Check if user is logged in and is admin
    if not session.get('logged_in') or not session.get('is_admin'):
        return redirect(url_for('welcome'))
    
    try:
        # This is a simplified example - in a real application, you would use a proper
        # database backup tool or library to create a SQL dump
        from datetime import datetime
        import subprocess
        import tempfile
        import os
        
        # Create a temporary file for the backup
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_filename = f"database_backup_{timestamp}.sql"
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.sql')
        temp_file.close()
        
        # Use mysqldump to create a backup
        # Note: In a production environment, you should use more secure methods to handle credentials
        subprocess.run([
            'mysqldump',
            '--user=root',
            '--password=root',
            'maincontroller',
            '--result-file=' + temp_file.name
        ], check=True)
        
        # Read the backup file
        with open(temp_file.name, 'rb') as f:
            backup_data = f.read()
        
        # Clean up the temporary file
        os.unlink(temp_file.name)
        
        # Return the backup as a downloadable file
        from flask import send_file
        from io import BytesIO
        
        return send_file(
            BytesIO(backup_data),
            mimetype='application/sql',
            as_attachment=True,
            download_name=backup_filename
        )
    except Exception as e:
        flash(f'Error creating backup: {str(e)}', 'error')
        return redirect(url_for('system_setup'))

@app.route('/restore_database', methods=['POST'])
def restore_database():
    # Check if user is logged in and is admin
    if not session.get('logged_in') or not session.get('is_admin'):
        return redirect(url_for('welcome'))
    
    try:
        # Check if a file was uploaded
        if 'backupFile' not in request.files:
            flash('No file selected', 'error')
            return redirect(url_for('system_setup'))
        
        backup_file = request.files['backupFile']
        
        # Check if the file has a name
        if backup_file.filename == '':
            flash('No file selected', 'error')
            return redirect(url_for('system_setup'))
        
        # Check if the file is a SQL file
        if not backup_file.filename.endswith('.sql'):
            flash('Invalid file type. Please upload a .sql file', 'error')
            return redirect(url_for('system_setup'))
        
        # Save the file temporarily
        import tempfile
        import os
        import subprocess
        
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.sql')
        backup_file.save(temp_file.name)
        temp_file.close()
        
        # Use mysql to restore the database
        # Note: In a production environment, you should use more secure methods to handle credentials
        subprocess.run([
            'mysql',
            '--user=root',
            '--password=root',
            'maincontroller',
            '-e', f'source {temp_file.name}'
        ], check=True)
        
        # Clean up the temporary file
        os.unlink(temp_file.name)
        
        flash('Database restored successfully', 'success')
    except Exception as e:
        flash(f'Error restoring database: {str(e)}', 'error')
    
    return redirect(url_for('system_setup'))

@app.route('/health')  
def health():
    """Simple endpoint to verify the server is running."""
    return jsonify(status='ok', message='Flask server running'), 200

# Part Number Management Routes
@app.route('/process_creation')
def process_creation():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    # Get all active part numbers with their related data
    part_numbers = db.session.query(PartNumber).filter_by(active=True).all()
    
    # For each part number, get related tapes, stamps, and insertions
    part_data = []
    for part in part_numbers:
        tapes = Tape.query.filter_by(part_number_id=part.id, active=True).all()
        stamps = Stamp.query.filter_by(part_number_id=part.id, active=True).all()
        insertions = Insertion.query.filter_by(part_number_id=part.id, active=True).all()
        
        part_data.append({
            'part': part,
            'tapes': tapes,
            'stamps': stamps,
            'insertions': insertions
        })
    
    return render_template('process_creation.html', username=username, language=language, part_data=part_data)

@app.route('/create_part_number', methods=['POST'])
def create_part_number():
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        data = request.get_json()
        username = session.get('username')
        
        # Validate required fields
        if not data.get('part_number') or not data.get('name') or not data.get('total_length'):
            return jsonify(success=False, message='Part number, name, and total length are required'), 400
        
        # Check if part number already exists
        existing_part = PartNumber.query.filter_by(part_number=data['part_number']).first()
        if existing_part:
            return jsonify(success=False, message='Part number already exists'), 400
        
        # Create new part number
        new_part = PartNumber(
            part_number=data['part_number'],
            name=data['name'],
            total_length=float(data['total_length']),
            created_by=username
        )
        db.session.add(new_part)
        db.session.flush()  # Get the ID
        
        # Add tapes if provided
        if data.get('tapes'):
            for tape_data in data['tapes']:
                tape = Tape(
                    part_number_id=new_part.id,
                    color=tape_data['color'],
                    width=float(tape_data['width']),
                    position=float(tape_data['position'])
                )
                db.session.add(tape)
        
        # Add stamps if provided
        if data.get('stamps'):
            for stamp_data in data['stamps']:
                stamp = Stamp(
                    part_number_id=new_part.id,
                    color=stamp_data['color'],
                    position=float(stamp_data['position'])
                )
                db.session.add(stamp)
        
        # Add insertions if provided
        if data.get('insertions'):
            for insertion_data in data['insertions']:
                insertion = Insertion(
                    part_number_id=new_part.id,
                    side=insertion_data['side'],
                    component=insertion_data['component']
                )
                db.session.add(insertion)
        
        db.session.commit()
        return jsonify(success=True, message='Part number created successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/update_part_number/<int:part_id>', methods=['POST'])
def update_part_number(part_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        data = request.get_json()
        
        # Find the part number
        part = PartNumber.query.get(part_id)
        if not part:
            return jsonify(success=False, message='Part number not found'), 404
        
        # Update part number details
        if data.get('name'):
            part.name = data['name']
        if data.get('total_length'):
            part.total_length = float(data['total_length'])
        
        # Update tapes - remove existing and add new ones
        if 'tapes' in data:
            # Deactivate existing tapes
            existing_tapes = Tape.query.filter_by(part_number_id=part_id).all()
            for tape in existing_tapes:
                tape.active = False
            
            # Add new tapes
            for tape_data in data['tapes']:
                tape = Tape(
                    part_number_id=part_id,
                    color=tape_data['color'],
                    width=float(tape_data['width']),
                    position=float(tape_data['position'])
                )
                db.session.add(tape)
        
        # Update stamps - remove existing and add new ones
        if 'stamps' in data:
            # Deactivate existing stamps
            existing_stamps = Stamp.query.filter_by(part_number_id=part_id).all()
            for stamp in existing_stamps:
                stamp.active = False
            
            # Add new stamps
            for stamp_data in data['stamps']:
                stamp = Stamp(
                    part_number_id=part_id,
                    color=stamp_data['color'],
                    position=float(stamp_data['position'])
                )
                db.session.add(stamp)
        
        # Update insertions - remove existing and add new ones
        if 'insertions' in data:
            # Deactivate existing insertions
            existing_insertions = Insertion.query.filter_by(part_number_id=part_id).all()
            for insertion in existing_insertions:
                insertion.active = False
            
            # Add new insertions
            for insertion_data in data['insertions']:
                insertion = Insertion(
                    part_number_id=part_id,
                    side=insertion_data['side'],
                    component=insertion_data['component']
                )
                db.session.add(insertion)
        
        part.updated_at = datetime.utcnow()
        db.session.commit()
        return jsonify(success=True, message='Part number updated successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/delete_part_number/<int:part_id>', methods=['POST'])
def delete_part_number(part_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        # Find the part number
        part = PartNumber.query.get(part_id)
        if not part:
            return jsonify(success=False, message='Part number not found'), 404
        
        # Soft delete - set active to False
        part.active = False
        
        # Also deactivate related records
        tapes = Tape.query.filter_by(part_number_id=part_id).all()
        for tape in tapes:
            tape.active = False
        
        stamps = Stamp.query.filter_by(part_number_id=part_id).all()
        for stamp in stamps:
            stamp.active = False
        
        insertions = Insertion.query.filter_by(part_number_id=part_id).all()
        for insertion in insertions:
            insertion.active = False
        
        db.session.commit()
        return jsonify(success=True, message='Part number deleted successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/get_part_number/<int:part_id>')
def get_part_number(part_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        # Find the part number
        part = PartNumber.query.get(part_id)
        if not part:
            return jsonify(success=False, message='Part number not found'), 404
        
        # Get related data
        tapes = Tape.query.filter_by(part_number_id=part_id, active=True).all()
        stamps = Stamp.query.filter_by(part_number_id=part_id, active=True).all()
        insertions = Insertion.query.filter_by(part_number_id=part_id, active=True).all()
        
        return jsonify(
            success=True,
            part={
                'id': part.id,
                'part_number': part.part_number,
                'name': part.name,
                'total_length': part.total_length,
                'created_by': part.created_by,
                'created_at': part.created_at.isoformat(),
                'updated_at': part.updated_at.isoformat(),
                'tapes': [{'color': t.color, 'width': t.width, 'position': t.position} for t in tapes],
                'stamps': [{'color': s.color, 'position': s.position} for s in stamps],
                'insertions': [{'side': i.side, 'component': i.component} for i in insertions]
            }
        )
        
    except Exception as e:
        return jsonify(success=False, message=str(e)), 500

@app.route('/get_part_numbers', methods=['GET'])
def get_part_numbers():
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        # Get all part numbers
        part_numbers = PartNumber.query.filter_by(active=True).all()
        
        return jsonify(
            success=True,
            part_numbers=[{
                'id': p.id,
                'part_number': p.part_number,
                'name': p.name
            } for p in part_numbers]
        )
        
    except Exception as e:
        return jsonify(success=False, message=str(e)), 500

@app.route('/get_bom_data/<int:part_number_id>', methods=['GET'])
def get_bom_data(part_number_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        # Get the part number
        part_number = PartNumber.query.get(part_number_id)
        if not part_number:
            return jsonify(success=False, message='Part number not found'), 404
        
        # Get associated components
        tapes = Tape.query.filter_by(part_number_id=part_number_id, active=True).all()
        stamps = Stamp.query.filter_by(part_number_id=part_number_id, active=True).all()
        insertions = Insertion.query.filter_by(part_number_id=part_number_id, active=True).all()
        
        # Get nozzles and joints (assuming they're linked through insertions or other means)
        # For now, we'll get all inactive nozzles and joints (active=0) as they might be used with any part
        nozzles = Nozzle.query.filter_by(active=True).all()
        joints = Joint.query.filter_by(active=True).all()
        
        bom_data = []
        
        # Add tapes to BOM
        for tape in tapes:
            bom_data.append({
                'type': 'tape',
                'part_tool': f'Tape - {tape.color}',
                'description': f'Width: {tape.width}mm, Position: {tape.position}',
                'quantity': 1
            })
        
        # Add stamps to BOM
        for stamp in stamps:
            bom_data.append({
                'type': 'stamp',
                'part_tool': f'Stamp - {stamp.color}',
                'description': f'Position: {stamp.position}',
                'quantity': 1
            })
        
        # Add insertions to BOM
        for insertion in insertions:
            bom_data.append({
                'type': 'insertion',
                'part_tool': f'Insertion - {insertion.component}',
                'description': f'Side: {insertion.side}',
                'quantity': 1
            })
        
        # Add nozzles to BOM (if applicable to this part)
        for nozzle in nozzles:
            bom_data.append({
                'type': 'nozzle',
                'part_tool': f'Nozzle - {nozzle.model}',
                'description': nozzle.description or 'Standard nozzle',
                'quantity': 1
            })
        
        # Add joints to BOM (if applicable to this part)
        for joint in joints:
            bom_data.append({
                'type': 'joint',
                'part_tool': f'Joint - {joint.model}',
                'description': joint.description or 'Standard joint',
                'quantity': 1
            })
        
        return jsonify(
            success=True,
            part_number={
                'id': part_number.id,
                'part_number': part_number.part_number,
                'name': part_number.name
            },
            bom_data=bom_data
        )
        
    except Exception as e:
        return jsonify(success=False, message=str(e)), 500

@app.route('/details_management')
def details_management():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    # Get all active nozzles and joints
    nozzles = Nozzle.query.filter_by(active=True).all()
    joints = Joint.query.filter_by(active=True).all()
    
    return render_template('details_management.html', 
                           username=username, 
                           language=language,
                           nozzles=nozzles,
                           joints=joints)

@app.route('/create_nozzle', methods=['POST'])
def create_nozzle():
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        data = request.get_json()
        username = session.get('username')
        
        # Create new nozzle
        new_nozzle = Nozzle(
            model=data['model'],
            description=data.get('description', ''),
            created_by=username
        )
        db.session.add(new_nozzle)
        db.session.commit()
        
        return jsonify(success=True, message='Nozzle created successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/update_nozzle/<int:nozzle_id>', methods=['POST'])
def update_nozzle(nozzle_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        data = request.get_json()
        
        # Find the nozzle
        nozzle = Nozzle.query.get(nozzle_id)
        if not nozzle:
            return jsonify(success=False, message='Nozzle not found'), 404
        
        # Update nozzle details
        nozzle.model = data['model']
        nozzle.description = data.get('description', '')
        nozzle.updated_at = datetime.utcnow()
        
        db.session.commit()
        return jsonify(success=True, message='Nozzle updated successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/delete_nozzle/<int:nozzle_id>', methods=['DELETE'])
def delete_nozzle(nozzle_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        # Find the nozzle
        nozzle = Nozzle.query.get(nozzle_id)
        if not nozzle:
            return jsonify(success=False, message='Nozzle not found'), 404
        
        # Soft delete - set active to False
        nozzle.active = False
        db.session.commit()
        
        return jsonify(success=True, message='Nozzle deleted successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/create_joint', methods=['POST'])
def create_joint():
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        data = request.get_json()
        username = session.get('username')
        
        # Create new joint
        new_joint = Joint(
            model=data['model'],
            description=data.get('description', ''),
            created_by=username
        )
        db.session.add(new_joint)
        db.session.commit()
        
        return jsonify(success=True, message='Joint created successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/update_joint/<int:joint_id>', methods=['POST'])
def update_joint(joint_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        data = request.get_json()
        
        # Find the joint
        joint = Joint.query.get(joint_id)
        if not joint:
            return jsonify(success=False, message='Joint not found'), 404
        
        # Update joint details
        joint.model = data['model']
        joint.description = data.get('description', '')
        joint.updated_at = datetime.utcnow()
        
        db.session.commit()
        return jsonify(success=True, message='Joint updated successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/delete_joint/<int:joint_id>', methods=['DELETE'])
def delete_joint(joint_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        # Find the joint
        joint = Joint.query.get(joint_id)
        if not joint:
            return jsonify(success=False, message='Joint not found'), 404
        
        # Soft delete - set active to False
        joint.active = False
        db.session.commit()
        
        return jsonify(success=True, message='Joint deleted successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/get_nozzles', methods=['GET'])
def get_nozzles():
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        # Get all active nozzles
        nozzles = Nozzle.query.filter_by(active=True).all()
        
        return jsonify(
            success=True,
            nozzles=[{
                'id': n.id,
                'model': n.model,
                'description': n.description
            } for n in nozzles]
        )
        
    except Exception as e:
        return jsonify(success=False, message=str(e)), 500

@app.route('/get_joints', methods=['GET'])
def get_joints():
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not logged in'), 401
    
    try:
        # Get all active joints
        joints = Joint.query.filter_by(active=True).all()
        
        return jsonify(
            success=True,
            joints=[{
                'id': j.id,
                'model': j.model,
                'description': j.description
            } for j in joints]
        )
        
    except Exception as e:
        return jsonify(success=False, message=str(e)), 500

# Device Management Routes
@app.route('/device_management')
def device_management():
    if 'username' not in session:
        return redirect(url_for('welcome'))
    
    # Get user language preference
    user_pref = UserPreference.query.filter_by(username=session['username']).first()
    language = user_pref.language if user_pref else 'en'
    
    # Get all active devices
    devices = Device.query.filter_by(active=True).all()
    
    return render_template('device_management.html', 
                         username=session['username'], 
                         language=language,
                         devices=devices)

@app.route('/create_device', methods=['POST'])
def create_device():
    if 'username' not in session:
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        data = request.get_json()
        
        # Check if canbus_id already exists
        existing_device = Device.query.filter_by(canbus_id=data['canbus_id'], active=True).first()
        if existing_device:
            return jsonify(success=False, message='CAN Bus ID already exists'), 400
        
        # Verify device type exists
        device_type_id = data.get('device_type_id')
        if device_type_id:
            device_type = DeviceType.query.get(device_type_id)
            if not device_type or not device_type.active:
                return jsonify(success=False, message='Invalid device type'), 400
        
        device = Device(
            name=data['name'],
            canbus_id=data['canbus_id'],
            device_type_id=device_type_id,
            created_by=session['username']
        )
        
        db.session.add(device)
        db.session.commit()
        
        return jsonify(success=True, message='Device created successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/update_device/<int:device_id>', methods=['POST'])
def update_device(device_id):
    if 'username' not in session:
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        data = request.get_json()
        device = Device.query.get_or_404(device_id)
        
        # Verify device type exists
        device_type_id = data.get('device_type_id')
        if device_type_id:
            device_type = DeviceType.query.get(device_type_id)
            if not device_type or not device_type.active:
                return jsonify(success=False, message='Invalid device type'), 400
        
        # Check if canbus_id already exists (excluding current device)
        existing_device = Device.query.filter(
            Device.canbus_id == data['canbus_id'],
            Device.id != device_id,
            Device.active == True
        ).first()
        
        if existing_device:
            return jsonify(success=False, message='CAN Bus ID already exists'), 400
        
        device.name = data['name']
        device.canbus_id = data['canbus_id']
        device.device_type_id = device_type_id
        device.updated_at = datetime.utcnow()
        
        db.session.commit()
        
        return jsonify(success=True, message='Device updated successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/delete_device/<int:device_id>', methods=['DELETE'])
def delete_device(device_id):
    if 'username' not in session:
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        device = Device.query.get_or_404(device_id)
        device.active = False
        device.updated_at = datetime.utcnow()
        
        db.session.commit()
        
        return jsonify(success=True, message='Device deleted successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/get_devices', methods=['GET'])
def get_devices():
    if 'username' not in session:
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        devices = Device.query.filter_by(active=True).all()
        
        return jsonify(
            success=True,
            devices=[{
                'id': d.id,
                'name': d.name,
                'canbus_id': d.canbus_id,
                'device_type_id': d.device_type_id,
                'device_type_name': d.device_type.name if d.device_type else 'Unknown'
            } for d in devices]
        )
        
    except Exception as e:
        return jsonify(success=False, message=str(e)), 500

# Device Type Management Routes
@app.route('/get_device_types', methods=['GET'])
def get_device_types():
    if 'username' not in session:
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        device_types = DeviceType.query.filter_by(active=True).all()
        
        return jsonify([{
            'id': dt.id,
            'name': dt.name,
            'description': dt.description,
            'created_by': dt.created_by,
            'created_at': dt.created_at.isoformat()
        } for dt in device_types])
        
    except Exception as e:
        return jsonify(success=False, message=str(e)), 500

@app.route('/create_device_type', methods=['POST'])
def create_device_type():
    if 'username' not in session:
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        name = request.form.get('name')
        description = request.form.get('description')
        
        if not name:
            return jsonify(success=False, message='Name is required'), 400
        
        # Check if device type name already exists
        existing_device_type = DeviceType.query.filter_by(name=name, active=True).first()
        if existing_device_type:
            return jsonify(success=False, message='Device type name already exists'), 400
        
        device_type = DeviceType(
            name=name,
            description=description,
            created_by=session['username']
        )
        
        db.session.add(device_type)
        db.session.commit()
        
        return jsonify(success=True, message='Device type created successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/update_device_type/<int:device_type_id>', methods=['POST'])
def update_device_type(device_type_id):
    if 'username' not in session:
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        device_type = DeviceType.query.get_or_404(device_type_id)
        
        name = request.form.get('name')
        description = request.form.get('description')
        
        if not name:
            return jsonify(success=False, message='Name is required'), 400
        
        # Check if device type name already exists (excluding current device type)
        existing_device_type = DeviceType.query.filter(
            DeviceType.name == name,
            DeviceType.active == True,
            DeviceType.id != device_type_id
        ).first()
        
        if existing_device_type:
            return jsonify(success=False, message='Device type name already exists'), 400
        
        device_type.name = name
        device_type.description = description
        device_type.updated_at = datetime.utcnow()
        
        db.session.commit()
        
        return jsonify(success=True, message='Device type updated successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/delete_device_type/<int:device_type_id>', methods=['DELETE'])
def delete_device_type(device_type_id):
    if 'username' not in session:
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        device_type = DeviceType.query.get_or_404(device_type_id)
        
        # Check if device type is being used by any devices
        devices_using_type = Device.query.filter_by(device_type_id=device_type_id, active=True).first()
        if devices_using_type:
            return jsonify(success=False, message='Cannot delete device type that is being used by devices'), 400
        
        # Soft delete
        device_type.active = False
        device_type.updated_at = datetime.utcnow()
        
        db.session.commit()
        
        return jsonify(success=True, message='Device type deleted successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

# Device Function Management Routes
@app.route('/get_device_functions/<int:device_type_id>', methods=['GET'])
def get_device_functions(device_type_id):
    if 'username' not in session:
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        device_functions = DeviceFunction.query.filter_by(device_type_id=device_type_id, active=True).all()
        
        return jsonify(
            success=True,
            functions=[{
                'id': df.id,
                'function_id': df.function_id,
                'description': df.description,
                'bit0': df.bit0,
                'bit1': df.bit1,
                'bit2': df.bit2,
                'bit3': df.bit3,
                'bit4': df.bit4,
                'bit5': df.bit5,
                'bit6': df.bit6,
                'bit7': df.bit7
            } for df in device_functions]
        )
        
    except Exception as e:
        return jsonify(success=False, message=str(e)), 500


# Tooling Management Routes
@app.route('/tooling_management')
def tooling_management():
    if 'username' not in session:
        return redirect(url_for('welcome'))
    
    # Get user language preference
    user_pref = UserPreference.query.filter_by(username=session['username']).first()
    language = user_pref.language if user_pref else 'en'
    
    return render_template('tooling_management.html', 
                         username=session['username'], 
                         language=language)

@app.route('/create_tool', methods=['POST'])
def create_tool():
    try:
        data = request.get_json()
        
        new_tool = Tool(
            part_number_id=data['part_number_id'],
            tool_number=data['tool_number'],
            description=data.get('description', ''),
            tool_type=data.get('tool_type', ''),
            quantity=data.get('quantity', 1),
            active=True,
            created_by=session.get('username', 'system')
        )
        
        db.session.add(new_tool)
        db.session.commit()
        
        return jsonify({
            'success': True,
            'message': 'Tool created successfully',
            'tool': {
                'id': new_tool.id,
                'part_number_id': new_tool.part_number_id,
                'tool_number': new_tool.tool_number,
                'description': new_tool.description,
                'tool_type': new_tool.tool_type,
                'quantity': new_tool.quantity,
                'active': new_tool.active
            }
        })
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/update_tool/<int:tool_id>', methods=['POST'])
def update_tool(tool_id):
    try:
        tool = Tool.query.get_or_404(tool_id)
        data = request.get_json()
        
        tool.part_number_id = data['part_number_id']
        tool.tool_number = data['tool_number']
        tool.description = data.get('description', '')
        tool.tool_type = data.get('tool_type', '')
        tool.quantity = data.get('quantity', 1)
        
        db.session.commit()
        
        return jsonify({
            'success': True,
            'message': 'Tool updated successfully',
            'tool': {
                'id': tool.id,
                'part_number_id': tool.part_number_id,
                'tool_number': tool.tool_number,
                'description': tool.description,
                'tool_type': tool.tool_type,
                'quantity': tool.quantity,
                'active': tool.active
            }
        })
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/delete_tool/<int:tool_id>', methods=['DELETE'])
def delete_tool(tool_id):
    try:
        tool = Tool.query.get_or_404(tool_id)
        tool.active = False
        
        db.session.commit()
        
        return jsonify(success=True, message='Tool deactivated successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/toggle_tool_status/<int:tool_id>', methods=['POST'])
def toggle_tool_status(tool_id):
    try:
        tool = Tool.query.get_or_404(tool_id)
        tool.active = not tool.active
        
        db.session.commit()
        
        status_text = 'activated' if tool.active else 'deactivated'
        return jsonify(success=True, message=f'Tool {status_text} successfully')
        
    except Exception as e:
        db.session.rollback()
        return jsonify(success=False, message=str(e)), 500

@app.route('/get_tools', methods=['GET'])
def get_tools():
    try:
        tools = Tool.query.filter_by(active=True).all()
        
        return jsonify(
            success=True,
            tools=[{
                'id': tool.id,
                'part_number_id': tool.part_number_id,
                'part_number': tool.part_number.part_number if tool.part_number else '',
                'tool_number': tool.tool_number,
                'description': tool.description,
                'tool_type': tool.tool_type,
                'quantity': tool.quantity,
                'active': tool.active
            } for tool in tools]
        )
        
    except Exception as e:
        return jsonify(success=False, message=str(e)), 500

@app.route('/get_tools_by_part/<int:part_number_id>', methods=['GET'])
def get_tools_by_part(part_number_id):
    try:
        tools = Tool.query.filter_by(part_number_id=part_number_id, active=True).all()
        
        return jsonify(
            success=True,
            tools=[{
                'id': tool.id,
                'tool_number': tool.tool_number,
                'description': tool.description,
                'tool_type': tool.tool_type,
                'quantity': tool.quantity
            } for tool in tools]
        )
        
    except Exception as e:
        return jsonify(success=False, message=str(e)), 500


@app.route('/work_orders')
def work_orders():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user language preference
    username = session.get('username')
    user = User.query.filter_by(username=username).first()
    language = 'en'  # default
    if user:
        user_pref = UserPreference.query.filter_by(username=username).first()
        if user_pref:
            language = user_pref.language
    
    # Get all active work orders (not deleted)
    work_orders = WorkOrder.query.filter_by(deleted=False).order_by(WorkOrder.created.desc()).all()
    
    # Get all part numbers for the dropdown
    part_numbers = PartNumber.query.all()
    
    return render_template('work_orders.html', 
                         username=username, 
                         language=language,
                         work_orders=work_orders,
                         part_numbers=part_numbers)

@app.route('/create_work_order', methods=['POST'])
def create_work_order():
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'})
    
    try:
        data = request.get_json()
        username = session.get('username')
        
        # Create new work order
        work_order = WorkOrder(
            part_number=data['part_number'],
            qty=int(data['qty']),
            balance=int(data['balance']),
            type=data['type'],
            user=username,
            created=datetime.now()
        )
        
        # Set optional datetime fields if provided
        if data.get('started_time'):
            work_order.started_time = datetime.fromisoformat(data['started_time'])
        if data.get('end_time'):
            work_order.end_time = datetime.fromisoformat(data['end_time'])
        
        db.session.add(work_order)
        db.session.commit()
        
        return jsonify({'success': True, 'message': 'Work order created successfully'})
    
    except Exception as e:
        db.session.rollback()
        return jsonify({'success': False, 'message': str(e)})

@app.route('/get_work_order/<int:work_order_id>')
def get_work_order(work_order_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'})
    
    try:
        work_order = WorkOrder.query.get_or_404(work_order_id)
        
        work_order_data = {
            'id': work_order.id,
            'part_number': work_order.part_number,
            'qty': work_order.qty,
            'balance': work_order.balance,
            'type': work_order.type,
            'user': work_order.user,
            'created': work_order.created.isoformat() if work_order.created else None,
            'started_time': work_order.started_time.isoformat() if work_order.started_time else None,
            'end_time': work_order.end_time.isoformat() if work_order.end_time else None
        }
        
        return jsonify({'success': True, 'work_order': work_order_data})
    
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/update_work_order/<int:work_order_id>', methods=['POST'])
def update_work_order(work_order_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'})
    
    try:
        data = request.get_json()
        work_order = WorkOrder.query.get_or_404(work_order_id)
        
        # Update work order fields
        work_order.part_number = data['part_number']
        work_order.qty = int(data['qty'])
        work_order.balance = int(data['balance'])
        work_order.type = data['type']
        
        # Update optional datetime fields
        if data.get('started_time'):
            work_order.started_time = datetime.fromisoformat(data['started_time'])
        else:
            work_order.started_time = None
            
        if data.get('end_time'):
            work_order.end_time = datetime.fromisoformat(data['end_time'])
        else:
            work_order.end_time = None
        
        db.session.commit()
        
        return jsonify({'success': True, 'message': 'Work order updated successfully'})
    
    except Exception as e:
        db.session.rollback()
        return jsonify({'success': False, 'message': str(e)})

@app.route('/delete_work_order/<int:work_order_id>', methods=['POST'])
def delete_work_order(work_order_id):
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'})
    
    try:
        work_order = WorkOrder.query.get_or_404(work_order_id)
        username = session.get('username')
        
        # Soft delete - mark as deleted instead of actually deleting
        work_order.deleted = True
        work_order.deleted_at = datetime.now()
        work_order.deleted_user = username
        
        db.session.commit()
        
        return jsonify({'success': True, 'message': 'Work order deleted successfully'})
    
    except Exception as e:
        db.session.rollback()
        return jsonify({'success': False, 'message': str(e)})

@app.route('/capacity_and_rate')
def capacity_and_rate():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    # Get unique part numbers for dropdown
    part_numbers = PartNumber.query.filter_by(active=True).all()

    
    return render_template('capacity_and_rate.html', 
                         username=username, 
                         language=language,
                         part_numbers=part_numbers)

@app.route('/preventive_maintenance')
def preventive_maintenance():
    # Check if user is logged in
    if not session.get('logged_in'):
        return redirect(url_for('welcome'))
    
    # Get user's language preference
    username = session.get('username')
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    # Check if language is provided in URL parameters
    url_lang = request.args.get('lang')
    if url_lang and url_lang in ['en', 'es', 'jp']:
        language = url_lang
        # Update user preference if it exists
        if user_pref:
            user_pref.language = language
            db.session.commit()
    elif user_pref:
        language = user_pref.language
    
    # Get pagination parameters
    page = request.args.get('page', 1, type=int)
    per_page = request.args.get('per_page', 10, type=int)
    selected_date = request.args.get('selected_date', '')
    
    # Ensure per_page is within reasonable limits
    per_page = max(5, min(per_page, 100))
    
    # Get all devices with their preventive maintenance data and device types
    devices_query = db.session.query(Device, PreventiveMaintenance, DeviceType).outerjoin(
        PreventiveMaintenance, Device.id == PreventiveMaintenance.device_id
    ).join(DeviceType, Device.device_type_id == DeviceType.id).filter(
        Device.active == True
    )
    
    # Filter by selected date if provided
    if selected_date:
        try:
            selected_date_obj = datetime.strptime(selected_date, '%Y-%m-%d').date()
            devices_query = devices_query.filter(
                PreventiveMaintenance.next_pm == selected_date_obj
            )
        except ValueError:
            selected_date = ''
    
    # Get paginated results
    devices_with_pm = devices_query.paginate(
        page=page, per_page=per_page, error_out=False
    )
    
    # Calculate status for each device
    devices_data = []
    all_devices_data = []  # For calendar highlighting
    
    # Get all devices for calendar (not paginated)
    all_devices_with_pm = db.session.query(Device, PreventiveMaintenance, DeviceType).outerjoin(
        PreventiveMaintenance, Device.id == PreventiveMaintenance.device_id
    ).join(DeviceType, Device.device_type_id == DeviceType.id).filter(
        Device.active == True
    ).all()
    
    for device, pm, device_type in all_devices_with_pm:
        status = 'good'
        if pm:
            # Calculate percentage of lifetime used
            percentage = (pm.counter / pm.lifetime) * 100 if pm.lifetime > 0 else 0
            if percentage >= 90:
                status = 'critical'
            elif percentage >= 75:
                status = 'warning'
            
            # Check if next PM is overdue
            if pm.next_pm and pm.next_pm < date.today():
                status = 'overdue'
        
        all_devices_data.append({
            'device': device,
            'pm': pm,
            'device_type': device_type,
            'status': status
        })
    
    # Process paginated devices
    for device, pm, device_type in devices_with_pm.items:
        status = 'good'
        if pm:
            # Calculate percentage of lifetime used
            percentage = (pm.counter / pm.lifetime) * 100 if pm.lifetime > 0 else 0
            if percentage >= 90:
                status = 'critical'
            elif percentage >= 75:
                status = 'warning'
            
            # Check if next PM is overdue
            if pm.next_pm and pm.next_pm < date.today():
                status = 'overdue'
        
        devices_data.append({
            'device': device,
            'pm': pm,
            'device_type': device_type,
            'status': status
        })
    
    return render_template('preventive_maintenance.html', 
                         username=username, 
                         language=language,
                         devices_data=devices_data,
                         all_devices_data=all_devices_data,
                         pagination=devices_with_pm,
                         current_page=page,
                         per_page=per_page,
                         selected_date=selected_date)

@app.route('/create_preventive_maintenance', methods=['POST'])
def create_preventive_maintenance():
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'}), 401
    
    try:
        data = request.get_json()
        device_id = data.get('device_id')
        lifetime = data.get('lifetime')
        next_pm_str = data.get('next_pm')
        
        if not all([device_id, lifetime]):
            return jsonify({'success': False, 'message': 'Missing required fields'}), 400
        
        # Check if PM record already exists
        existing_pm = PreventiveMaintenance.query.filter_by(device_id=device_id, active=True).first()
        if existing_pm:
            return jsonify({'success': False, 'message': 'Preventive maintenance record already exists'}), 400
        
        # Parse next PM date if provided
        next_pm = None
        if next_pm_str:
            next_pm = datetime.strptime(next_pm_str, '%Y-%m-%d').date()
        
        # Create new PM record
        pm = PreventiveMaintenance(
            device_id=device_id,
            counter=0,
            lifetime=int(lifetime),
            next_pm=next_pm,
            created_by=session.get('username')
        )
        
        db.session.add(pm)
        db.session.commit()
        
        return jsonify({'success': True, 'message': 'Preventive maintenance record created successfully'})
        
    except Exception as e:
        db.session.rollback()
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/update_preventive_maintenance/<int:pm_id>', methods=['POST'])
def update_preventive_maintenance(pm_id):
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'}), 401
    
    try:
        data = request.get_json()
        pm = PreventiveMaintenance.query.get_or_404(pm_id)
        
        # Update fields if provided
        if 'counter' in data:
            pm.counter = int(data['counter'])
        if 'lifetime' in data:
            pm.lifetime = int(data['lifetime'])
        if 'last_pm' in data and data['last_pm']:
            pm.last_pm = datetime.strptime(data['last_pm'], '%Y-%m-%d').date()
        if 'next_pm' in data and data['next_pm']:
            pm.next_pm = datetime.strptime(data['next_pm'], '%Y-%m-%d').date()
        
        pm.updated_at = datetime.utcnow()
        db.session.commit()
        
        return jsonify({'success': True, 'message': 'Preventive maintenance record updated successfully'})
        
    except Exception as e:
        db.session.rollback()
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/perform_maintenance/<int:device_id>', methods=['POST'])
def perform_maintenance(device_id):
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'}), 401
    
    try:
        pm = PreventiveMaintenance.query.filter_by(device_id=device_id, active=True).first()
        if not pm:
            return jsonify({'success': False, 'message': 'Preventive maintenance record not found'}), 404
        
        # Reset counter and update dates
        pm.counter = 0
        pm.last_pm = date.today()
        
        # Calculate next PM date (assuming monthly maintenance)
        pm.next_pm = date.today() + timedelta(days=30)
        pm.updated_at = datetime.utcnow()
        
        db.session.commit()
        
        return jsonify({'success': True, 'message': 'Maintenance performed successfully'})
        
    except Exception as e:
        db.session.rollback()
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/get_pm_calendar_data', methods=['GET'])
def get_pm_calendar_data():
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'}), 401
    
    try:
        # Get all PM records with next_pm dates
        pm_records = PreventiveMaintenance.query.filter(
            PreventiveMaintenance.active == True,
            PreventiveMaintenance.next_pm.isnot(None)
        ).all()
        
        calendar_data = {}
        for pm in pm_records:
            date_str = pm.next_pm.strftime('%Y-%m-%d')
            if date_str not in calendar_data:
                calendar_data[date_str] = []
            
            device = Device.query.get(pm.device_id)
            calendar_data[date_str].append({
                'device_name': device.name if device else 'Unknown Device',
                'device_id': pm.device_id
            })
        
        return jsonify({'success': True, 'data': calendar_data})
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/populate_sample_pm_data')
def populate_sample_pm_data():
    if not session.get('logged_in') or not session.get('is_admin'):
        return jsonify({'success': False, 'message': 'Admin access required'}), 403
    
    try:
        # Get all devices
        devices = Device.query.filter(Device.active == True).all()
        
        for device in devices:
            # Check if PM record already exists
            existing_pm = PreventiveMaintenance.query.filter_by(device_id=device.id, active=True).first()
            if not existing_pm:
                # Create sample PM data
                import random
                counter = random.randint(50, 140)
                lifetime = 150
                last_pm = date.today() - timedelta(days=random.randint(1, 30))
                next_pm = date.today() + timedelta(days=random.randint(1, 60))
                
                pm = PreventiveMaintenance(
                    device_id=device.id,
                    counter=counter,
                    lifetime=lifetime,
                    last_pm=last_pm,
                    next_pm=next_pm,
                    created_by=session.get('username')
                )
                
                db.session.add(pm)
        
        db.session.commit()
        return jsonify({'success': True, 'message': 'Sample PM data populated successfully'})
        
    except Exception as e:
        db.session.rollback()
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/get_maintenance_comments/<int:device_id>', methods=['GET'])
def get_maintenance_comments(device_id):
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'}), 401
    
    try:
        # Get the latest maintenance comment for the device
        comment = PreventiveMaintenanceComments.query.filter_by(
            device_id=device_id
        ).order_by(PreventiveMaintenanceComments.performed_at.desc()).first()
        
        if comment:
            return jsonify({
                'success': True,
                'data': {
                    'id': comment.id,
                    'comments': comment.comments,
                    'maintenance_completed': comment.maintenance_completed,
                    'next_maintenance_date': comment.next_maintenance_date.strftime('%Y-%m-%d') if comment.next_maintenance_date else '',
                    'errors': comment.errors,
                    'performed_by': comment.performed_by,
                    'performed_at': comment.performed_at.strftime('%Y-%m-%d %H:%M:%S')
                }
            })
        else:
            return jsonify({'success': True, 'data': None})
            
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/save_maintenance_comments', methods=['POST'])
def save_maintenance_comments():
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'}), 401
    
    try:
        data = request.get_json()
        device_id = data.get('device_id')
        comments = data.get('comments', '')
        maintenance_completed = data.get('maintenance_completed', False)
        next_maintenance_date_str = data.get('next_maintenance_date')
        errors = data.get('errors', '')
        
        if not device_id:
            return jsonify({'success': False, 'message': 'Device ID is required'}), 400
        
        # Parse next maintenance date if provided
        next_maintenance_date = None
        if next_maintenance_date_str:
            try:
                next_maintenance_date = datetime.strptime(next_maintenance_date_str, '%Y-%m-%d').date()
            except ValueError:
                return jsonify({'success': False, 'message': 'Invalid date format'}), 400
        
        # Create new maintenance comment record
        maintenance_comment = PreventiveMaintenanceComments(
            device_id=device_id,
            comments=comments,
            maintenance_completed=maintenance_completed,
            next_maintenance_date=next_maintenance_date,
            errors=errors,
            performed_by=session.get('username')
        )
        
        db.session.add(maintenance_comment)
        
        # Get the PreventiveMaintenance record to update error count if needed
        pm_record = PreventiveMaintenance.query.filter_by(
            device_id=device_id, active=True
        ).first()
        
        # If errors are reported, increment the error count
        if errors and errors.strip() and pm_record:
            pm_record.error_count += 1
        
        # If maintenance is completed, update the PreventiveMaintenance record
        if maintenance_completed and pm_record:
            pm_record.counter = 0  # Reset counter
            pm_record.last_pm = date.today()
            if next_maintenance_date:
                pm_record.next_pm = next_maintenance_date
            else:
                # Default to 30 days from now if no date specified
                pm_record.next_pm = date.today() + timedelta(days=30)
            pm_record.updated_at = datetime.utcnow()
        
        db.session.commit()
        
        return jsonify({
            'success': True, 
            'message': 'Maintenance comments saved successfully'
        })
        
    except Exception as e:
        db.session.rollback()
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/device_details/<int:device_id>')
def device_details(device_id):
    if 'username' not in session:
        return redirect(url_for('welcome'))
    
    # Check for language parameter in URL
    lang_param = request.args.get('lang')
    if lang_param and lang_param in ['en', 'es', 'jp']:
        # Update user language preference if valid language provided
        user_pref = UserPreference.query.filter_by(username=session['username']).first()
        if user_pref:
            user_pref.language = lang_param
            db.session.commit()
        language = lang_param
    else:
        # Get user language preference
        user_pref = UserPreference.query.filter_by(username=session['username']).first()
        language = user_pref.language if user_pref else 'en'
    
    # Get device with its type and preventive maintenance info
    device = db.session.query(Device, DeviceType, PreventiveMaintenance).outerjoin(
        DeviceType, Device.device_type_id == DeviceType.id
    ).outerjoin(
        PreventiveMaintenance, Device.id == PreventiveMaintenance.device_id
    ).filter(
        Device.id == device_id,
        Device.active == True
    ).first()
    
    if not device:
        return redirect(url_for('preventive_maintenance'))
    
    device_obj, device_type, pm = device
    
    # Get maintenance comments with errors for this device
    maintenance_comments = PreventiveMaintenanceComments.query.filter_by(
        device_id=device_id
    ).order_by(PreventiveMaintenanceComments.created_at.desc()).all()
    
    # Filter comments that have errors
    error_comments = [comment for comment in maintenance_comments if comment.errors and comment.errors.strip()]
    
    return render_template('device_details.html',
                         username=session['username'],
                         language=language,
                         device=device_obj,
                         device_type=device_type,
                         pm=pm,
                         maintenance_comments=maintenance_comments,
                         error_comments=error_comments,
                         today=date.today())

@app.route('/get_production_history', methods=['GET'])
def get_production_history():
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'})
    
    try:
        from datetime import datetime, timedelta
        
        # Get date parameters from request
        from_date_str = request.args.get('from_date')
        to_date_str = request.args.get('to_date')
        
        # Use provided dates or default to last 30 days
        if from_date_str and to_date_str:
            try:
                start_date = datetime.strptime(from_date_str, '%Y-%m-%d').date()
                end_date = datetime.strptime(to_date_str, '%Y-%m-%d').date()
            except ValueError:
                return jsonify({'success': False, 'message': 'Invalid date format. Use YYYY-MM-DD'})
        else:
            today = datetime.now().date()
            start_date = today - timedelta(days=29)  # Last 30 days
            end_date = today
        
        # Query production history data from ProdHis table
        production_history = ProdHis.query.filter(
            ProdHis.date >= start_date,
            ProdHis.date <= end_date,
            ProdHis.deleted.is_(None)
        ).order_by(ProdHis.date, ProdHis.timestamp).all()
        
        # Convert to list of dictionaries for JSON response
        data = []
        for record in production_history:
            data.append({
                'id': record.id,
                'machine': record.machine,
                'date': record.date.strftime('%Y-%m-%d'),
                'part_number': record.part_number,
                'qty': int(record.qty) if record.qty is not None else 0,
                'timestamp': record.timestamp.strftime('%Y-%m-%d %H:%M:%S') if record.timestamp else '',
                'user': record.user
            })
        
        return jsonify({'success': True, 'data': data})
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/get_production_data', methods=['GET'])
def get_production_data():
    # Check if user is logged in
    if not session.get('logged_in'):
        return jsonify({'success': False, 'message': 'Not logged in'})
    
    try:
        from datetime import datetime, timedelta
        from sqlalchemy import func
        
        # Get date parameters from request
        from_date_str = request.args.get('from_date')
        to_date_str = request.args.get('to_date')
        
        # Use provided dates or default to last 30 days
        if from_date_str and to_date_str:
            try:
                start_date = datetime.strptime(from_date_str, '%Y-%m-%d').date()
                end_date = datetime.strptime(to_date_str, '%Y-%m-%d').date()
            except ValueError:
                return jsonify({'success': False, 'message': 'Invalid date format. Use YYYY-MM-DD'})
        else:
            today = datetime.now().date()
            start_date = today - timedelta(days=29)  # Last 30 days
            end_date = today
        
        # Query daily production data from ProdHis table
        daily_production_query = db.session.query(
            ProdHis.date,
            func.sum(ProdHis.qty).label('total_qty')
        ).filter(
            ProdHis.date >= start_date,
            ProdHis.date <= end_date,
            ProdHis.deleted.is_(None)
        ).group_by(ProdHis.date).order_by(ProdHis.date).all()
        
        # Create daily data structure
        daily_data = {}
        for record in daily_production_query:
            # Ensure the quantity is stored as integer
            daily_data[record.date] = int(record.total_qty) if record.total_qty is not None else 0
        
        # Fill in missing dates with 0 production
        labels = []
        production_data = []
        capacity_data = []
        efficiency_data = []
        
        # Calculate number of days in the selected range
        date_range = (end_date - start_date).days + 1
        
        for i in range(date_range):
            date = start_date + timedelta(days=i)
            labels.append(date.strftime('%Y-%m-%d'))
            
            # Get actual production data or 0 if no data
            production_qty = daily_data.get(date, 0)
            # Ensure the value is an integer to prevent string concatenation issues
            production_qty = int(production_qty) if production_qty is not None else 0
            production_data.append(production_qty)
            
            # Set capacity based on typical production line capacity
            # This could be made configurable or stored in database
            capacity_qty = 200  # Standard daily capacity
            capacity_data.append(capacity_qty)
            
            # Calculate efficiency
            efficiency = (production_qty / capacity_qty) * 100 if capacity_qty > 0 else 0
            efficiency_data.append(round(min(efficiency, 100), 1))  # Cap at 100%
        
        # Query weekly production data based on selected date range
        weekly_labels = []
        weekly_production = []
        weekly_capacity = []
        weekly_efficiency = []
        
        # Calculate number of weeks in the selected range
        total_days = (end_date - start_date).days + 1
        num_weeks = min(max(1, total_days // 7), 8)  # At least 1 week, max 8 weeks
        
        for i in range(num_weeks):
            week_start = start_date + timedelta(days=i*7)
            week_end = min(week_start + timedelta(days=6), end_date)
            
            # Query production for this week
            week_production_query = db.session.query(
                func.sum(ProdHis.qty).label('total_qty')
            ).filter(
                ProdHis.date >= week_start,
                ProdHis.date <= week_end,
                ProdHis.deleted.is_(None)
            ).scalar()
            
            week_prod = week_production_query or 0
            days_in_week = (week_end - week_start).days + 1
            week_cap = 200 * days_in_week  # Daily capacity * days in week
            week_eff = (week_prod / week_cap) * 100 if week_cap > 0 else 0
            
            weekly_labels.append(f"{week_start.strftime('%m/%d')} - {week_end.strftime('%m/%d')}")
            weekly_production.append(week_prod)
            weekly_capacity.append(week_cap)
            weekly_efficiency.append(round(min(week_eff, 100), 1))
        
        # Query part number breakdown for the selected date range
        parts_production_query = db.session.query(
            ProdHis.part_number,
            func.sum(ProdHis.qty).label('total_qty')
        ).filter(
            ProdHis.date >= start_date,
            ProdHis.date <= end_date,
            ProdHis.deleted.is_(None)
        ).group_by(ProdHis.part_number).order_by(func.sum(ProdHis.qty).desc()).all()
        
        part_numbers = []
        part_production = []
        
        for record in parts_production_query:
            part_numbers.append(record.part_number)
            part_production.append(record.total_qty)
        
        # If no parts data, provide empty arrays
        if not part_numbers:
            part_numbers = ['No Data']
            part_production = [0]
        
        chart_data = {
            'daily': {
                'labels': labels,
                'production': production_data,
                'capacity': capacity_data,
                'efficiency': efficiency_data
            },
            'weekly': {
                'labels': weekly_labels,
                'production': weekly_production,
                'capacity': weekly_capacity,
                'efficiency': weekly_efficiency
            },
            'parts': {
                'labels': part_numbers,
                'production': part_production
            }
        }
        
        return jsonify({'success': True, 'data': chart_data})
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

if __name__ == '__main__':
    # Host 0.0.0.0 allows access from the local network if needed
    app.run(host='0.0.0.0', port=5000, debug=True)