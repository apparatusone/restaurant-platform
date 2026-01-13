from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base
from .config import conf
from urllib.parse import quote_plus

SQLALCHEMY_DATABASE_URL = f"mysql+pymysql://{conf.db_user}:{quote_plus(conf.db_password)}@{conf.db_host}:{conf.db_port}/{conf.db_name}?charset=utf8mb4"
engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    pool_pre_ping=True,      # Test connections before using them
    pool_recycle=3600,       # Recycle connections after 1 hour
    pool_size=5,             # Default is 5, but being explicit
    max_overflow=10,         # Allow up to 10 extra connections when pool is full
    echo=False               # Set to True for SQL debugging
)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
