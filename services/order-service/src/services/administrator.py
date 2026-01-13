from sqlalchemy.orm import Session
from fastapi import HTTPException
from enum import Enum


class TestDataType(str, Enum):
    ALL = "all"
    CUSTOMERS = "customers"
    FOOD = "food"
    PROMOTIONS = "promotions"
    REVIEWS = "reviews"
    ORDERS = "orders"
    TABLES = "tables"


def add_test_data(data_type: TestDataType):
    """
    Add test data to the database.
    """
    try:
        if data_type == TestDataType.ALL:
            # add all test data types in order
            data_types = [
                TestDataType.CUSTOMERS,
                TestDataType.FOOD, 
                TestDataType.PROMOTIONS,
                TestDataType.REVIEWS,
                TestDataType.TABLES,
                TestDataType.ORDERS
            ]
            
            for data in data_types:
                result = add_test_data(data)
                if not result:
                    raise HTTPException(
                        status_code=500,
                        detail=f"Failed to add test {data.value}"
                    )
            
            return {"message": "All test data added successfully"}
            
        elif data_type == TestDataType.CUSTOMERS:
            from scripts.add_test_customers import add_test_customers
            success = add_test_customers()
            message = "Test customers added successfully"
            
        elif data_type == TestDataType.FOOD:
            from scripts.add_test_food import add_test_food
            success = add_test_food()
            message = "Test food items added successfully"
            
        elif data_type == TestDataType.PROMOTIONS:
            from scripts.add_test_promotions import add_test_promotions
            success = add_test_promotions()
            message = "Test promotions added successfully"
            
        elif data_type == TestDataType.REVIEWS:
            from scripts.add_test_reviews import add_test_reviews
            success = add_test_reviews()
            message = "Test reviews added successfully"
            
        elif data_type == TestDataType.ORDERS:
            from scripts.add_test_orders import add_test_orders
            success = add_test_orders()
            message = "Test orders added successfully"

        elif data_type == TestDataType.TABLES:
            from scripts.add_test_tables import add_test_tables
            success = add_test_tables()
            message = "Test tables added successfully"
                    
        if not success:
            raise HTTPException(
                status_code=500, 
                detail=f"Failed to add test {data_type.value}"
            )
        return {"message": message}
        
    except ImportError as e:
        raise HTTPException(
            status_code=500, 
            detail=f"Test script not found for {data_type.value}: {str(e)}"
        )
    except Exception as e:
        raise HTTPException(
            status_code=500, 
            detail=f"Error adding test {data_type.value}: {str(e)}"
        )


def purge_database(db: Session):
    """
    This will delete ALL data from the database permanently!
    """
    try:
        from shared.dependencies.database import Base
        from sqlalchemy import text
        
        # Delete all data from tables
        for table in reversed(Base.metadata.sorted_tables):
            db.execute(table.delete())
        
        # reset auto-increment IDs
        for table in Base.metadata.sorted_tables:
            db.execute(text(f"ALTER TABLE {table.name} AUTO_INCREMENT = 1"))
        
        db.commit()
        return {"message": "Database purged and IDs reset successfully."}
        
    except Exception as e:
        db.rollback()
        raise HTTPException(
            status_code=500, 
            detail=f"Failed to purge database: {str(e)}"
        )