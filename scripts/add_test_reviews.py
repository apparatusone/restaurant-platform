#!/usr/bin/env python3
"""
Script to add test review data
"""

from api.dependencies.database import SessionLocal
from api.models.reviews import Reviews
from api.models.menu_items import MenuItem
from api.models import model_loader
from datetime import datetime


def add_test_reviews():
    model_loader.index()
    
    db = SessionLocal()
    
    try:
        # first check if menu items exist
        menu_items = db.query(MenuItem).limit(4).all()
        if len(menu_items) < 4:
            print("Not enough menu items found. Please run add_test_food.py first.")
            return False
        
        reviews_data = [
            {
                "menu_item_id": menu_items[0].id,
                "customer_name": "Jason Bourne",
                "rating": 5,
                "review_text": "Jesus Christ, it's..."
            },
            {
                "menu_item_id": menu_items[1].id,
                "customer_name": "One Punch Man",
                "rating": 4,
                "review_text": "Good, but I wanted a bigger discount"
            },
            {
                "menu_item_id": menu_items[0].id,
                "customer_name": "Peter Peterson",
                "rating": 3,
                "review_text": "It was okay."
            },
            {
                "menu_item_id": menu_items[3].id,
                "customer_name": "Vessel",
                "rating": 5,
                "review_text": "The taste of the divine"
            }
        ]
        
        created_reviews = []
        
        for review_data in reviews_data:
            review = Reviews(
                menu_item_id=review_data["menu_item_id"],
                customer_name=review_data["customer_name"],
                rating=review_data["rating"],
                review_text=review_data["review_text"],
                created_at=datetime.now()
            )
            
            db.add(review)
            db.flush()
            
            created_reviews.append({
                "id": review.id,
                "menu_item_id": review.menu_item_id,
                "customer_name": review.customer_name,
                "rating": review.rating,
                "review_text": review.review_text
            })
        
        db.commit()
        print(f"Successfully created {len(created_reviews)} test reviews")
        return True
        
    except Exception as e:
        db.rollback()
        print(f"Error adding test reviews: {e}")
        return False
    
    finally:
        db.close()


if __name__ == "__main__":
    add_test_reviews()