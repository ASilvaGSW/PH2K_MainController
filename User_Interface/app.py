from flask import Flask, render_template, request, redirect, url_for, session, jsonify, flash
from datetime import datetime, date, timedelta
import os

# Import database models
from models import db, User, UserPreference, ProdHis, initialize_database,PartNumber,Tape,Stamp,Insertion,Nozzle,Joint,Device,DeviceType,DeviceFunction,WorkOrder

app = Flask(__name__)
app.config['SECRET_KEY'] = 'gswrnd2025'
app.config['SQLALCHEMY_DATABASE_URI'] = 'mysql+pymysql://root:root@localhost/maincontroller'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

# Initialize database with app
db.init_app(app)

# Initialize the database when the app starts
with app.app_context():
    initialize_database(app)

@app.route('/', methods=['GET', 'POST'])
def welcome():
    if request.method == 'POST':
        username = request.form.get('userId')
        password = request.form.get('password')
        badge_id = request.form.get('badgeId')
        
        # Check if user exists based on username/password or badge ID
        user = None
        if badge_id:
            user = User.query.filter_by(badge_id=badge_id).first()
        else:
            user = User.query.filter_by(username=username, password=password).first()
        
        if user:
            # Store user info in session
            session['logged_in'] = True
            session['username'] = user.username
            session['is_admin'] = user.is_admin
            
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
    user_pref = UserPreference.query.filter_by(username=username).first()
    language = 'en'  # Default language
    
    if user_pref:
        language = user_pref.language
    
    return render_template('main.html', username=username, language=language)

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
    part_numbers = ProdHis.query.with_entities(ProdHis.part_number).distinct().all()
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
    new_user = User(username=username, password=password, badge_id=badge_id, is_admin=is_admin)
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

if __name__ == '__main__':
    # Host 0.0.0.0 allows access from the local network if needed
    app.run(host='0.0.0.0', port=5000, debug=True)