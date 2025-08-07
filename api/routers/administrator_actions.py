from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from ..dependencies.database import get_db
from ..services import administrator as admin_services
from ..services.administrator import TestDataType

router = APIRouter(
    tags=['Administrator Actions'],
    prefix="/admin",
)

@router.post("/add-test-data/{data_type}")
def add_test_data(data_type: TestDataType):
    """
    Add test data to the database.
    """
    return admin_services.add_test_data(data_type)


@router.delete("/purge-db")
def purge_database(db: Session = Depends(get_db)):
    """
    This will delete ALL data from the database permanently!
    """
    return admin_services.purge_database(db)