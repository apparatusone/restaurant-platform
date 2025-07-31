from fastapi import APIRouter, Depends, status, Response
from sqlalchemy.orm import Session
from enum import Enum
from ..dependencies.database import get_db

# holds common actions made by administrators

class TestDataType(str, Enum):
    CUSTOMERS = "customers"
    FOOD = "food"
    ORDERS = "orders"
    PAYMENT_METHOD = "payment_method"
    PROMOTIONS = "promotions"
    REVIEWS = "reviews"

router = APIRouter(
    tags=['Administrator Actions'],
    prefix="/administrator_actions",
)

@router.post("/add-test-data/{data_type}")
def add_test_data(data_type: TestDataType):
    """
    Add specified test data
    """
    try:
        if data_type == TestDataType.CUSTOMERS:
            from scripts.add_test_customers import add_test_customers
            success = add_test_customers()
            if success:
                return {"message": "Test customers added successfully"}
            else:
                return {"error": "Failed to add test customers"}
        
        elif data_type == TestDataType.FOOD:
            from scripts.add_test_food import add_all_test_food
            add_all_test_food()
            return {"message": "Test food items added successfully"}
        
        else:
            return {"message": f"Test data for {data_type.value} not implemented yet"}
    
    except Exception as e:
        return {"error": f"Failed to add test data."}


@router.delete("/purge-db")
def purge_database(db: Session = Depends(get_db)):
    # Administrator action to purge all data from the database
    try:
        from ..dependencies.database import Base
        
        for table in reversed(Base.metadata.sorted_tables):
            db.execute(table.delete())
        
        db.commit()
        return {"message": "Database purged successfully"}
    except Exception as e:
        db.rollback()
        return {"error": f"Failed to purge database: {str(e)}"}