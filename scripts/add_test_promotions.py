#!/usr/bin/env python3
"""
Script to add test promotion data
"""

from api.dependencies.database import SessionLocal
from api.models.promotions import Promotion
from api.models import model_loader
from datetime import datetime, timedelta


def add_test_promotions():
    model_loader.index()
    
    db = SessionLocal()
    
    try:
        promotions_data = [
            {
                "code": "SAVE10",
                "description": "Save 10% on your order",
                "discount_percent": 10,
                "expiration_date": datetime.now() + timedelta(days=30)
            },
            {
                "code": "UNCC25",
                "description": "Student discount - 15% off",
                "discount_percent": 15,
                "expiration_date": datetime.now() + timedelta(days=90)
            },
            {
                "code": "BASICALLYFREE",
                "description": "Save 99 Percent!",
                "discount_percent": 99,
                "expiration_date": datetime.now() + timedelta(days=60)
            }
        ]
        
        created_promotions = []
        
        for promo_data in promotions_data:
            promotion = Promotion(
                code=promo_data["code"],
                description=promo_data["description"],
                discount_percent=promo_data["discount_percent"],
                expiration_date=promo_data["expiration_date"],
                created_at=datetime.now()
            )
            
            db.add(promotion)
            db.flush()
            
            created_promotions.append({
                "id": promotion.id,
                "code": promotion.code,
                "description": promotion.description,
                "discount_percent": promotion.discount_percent,
                "expiration_date": promotion.expiration_date
            })
        
        db.commit()
        print(f"Successfully created {len(created_promotions)} test promotions")
        return True
        
    except Exception as e:
        db.rollback()
        print(f"Error adding test promotions: {e}")
        return False
    
    finally:
        db.close()


if __name__ == "__main__":
    add_test_promotions()