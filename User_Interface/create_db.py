from app import app, db


def main() -> None:
    """Create all database tables if they do not already exist."""
    with app.app_context():
        db.create_all()
        print("Database tables created (or already exist). ✅")


if __name__ == "__main__":
    main()