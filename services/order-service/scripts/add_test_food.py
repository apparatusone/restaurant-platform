#!/usr/bin/env python3
"""
Script to add all test food items to the database using raw SQL
"""

import sys
import os

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from shared.dependencies.database import SessionLocal
from sqlalchemy import text

# main dishes
MAIN_DISHES = [
    {
        "name": "Extravagangent Pizza",
        "description": "Pepperoni, ham, Italian sausage, beef, onions, green peppers, mushrooms, and black olives",
        "price": 16.99,
        "calories": 320,
        "food_category": "regular",
        "ingredients": [
            ("Pizza Dough", 1),
            ("Pizza Sauce", 3),
            ("Mozzarella Cheese", 4),
            ("Pepperoni", 3),
            ("Ham", 2),
            ("Italian Sausage", 2),
            ("Ground Beef", 2),
            ("Onions", 1),
            ("Green Peppers", 1),
            ("Mushrooms", 2),
            ("Black Olives", 1)
        ]
    },
    {
        "name": "Meaty Pizza",
        "description": "Pepperoni, ham, beef, Italian sausage, and extra cheese",
        "price": 15.99,
        "calories": 350,
        "food_category": "regular",
        "ingredients": [
            ("Pizza Dough", 1),
            ("Pizza Sauce", 3),
            ("Mozzarella Cheese", 5),
            ("Pepperoni", 4),
            ("Ham", 3),
            ("Ground Beef", 3),
            ("Italian Sausage", 3)
        ]
    },
    {
        "name": "Super Deluxe Pizza",
        "description": "Pepperoni, Italian sausage, green peppers, mushrooms, and onions",
        "price": 14.99,
        "calories": 290,
        "food_category": "regular",
        "ingredients": [
            ("Pizza Dough", 1),
            ("Pizza Sauce", 3),
            ("Mozzarella Cheese", 4),
            ("Pepperoni", 3),
            ("Italian Sausage", 2),
            ("Green Peppers", 2),
            ("Mushrooms", 2),
            ("Onions", 1)
        ]
    },
    {
        "name": "Hawaiian Pizza",
        "description": "Ham, pineapple, and extra cheese",
        "price": 13.99,
        "calories": 280,
        "food_category": "vegetarian",
        "ingredients": [
            ("Pizza Dough", 1),
            ("Pizza Sauce", 3),
            ("Mozzarella Cheese", 5),
            ("Ham", 4),
            ("Pineapple", 3)
        ]
    }
]

SIDE_ITEMS = [
    {
        "name": "Small Ranch Cup",
        "description": "Small cup of ranch dressing",
        "price": 0.25,
        "calories": 120,
        "food_category": "vegetarian",
        "ingredients": [
            ("Ranch Dressing", 1)
        ]
    },
    {
        "name": "Garden Salad",
        "description": "Fresh lettuce, tomatoes, red onions, and green peppers",
        "price": 6.99,
        "calories": 70,
        "food_category": "vegetarian",
        "ingredients": [
            ("Mixed Lettuce", 3),
            ("Tomatoes", 2),
            ("Red Onions", 1),
            ("Green Peppers", 1),
            ("Italian Dressing", 1)
        ]
    },
    {
        "name": "Chicken Caesar Salad",
        "description": "Caesar salad topped with grilled chicken breast",
        "price": 9.99,
        "calories": 280,
        "food_category": "regular",
        "ingredients": [
            ("Romaine Lettuce", 3),
            ("Grilled Chicken", 4),
            ("Parmesan Cheese", 2),
            ("Croutons", 1),
            ("Caesar Dressing", 2)
        ]
    }
]

BEVERAGES = [
    {
        "name": "Coke Bottle",
        "description": "12oz Coca-Cola bottle",
        "price": 2.50,
        "calories": 140,
        "food_category": "vegan",
        "ingredients": [
            ("Coke Bottle", 1)
        ]
    },
    {
        "name": "Water Bottle",
        "description": "16oz bottled water",
        "price": 1.99,
        "calories": 0,
        "food_category": "vegan",
        "ingredients": [
            ("Water Bottle", 1)
        ]
    }
]

def add_test_food():
    db = SessionLocal()
    
    try:
        # Combine all menu items
        menu_items_data = MAIN_DISHES + SIDE_ITEMS + BEVERAGES
        
        created_items = []
        
        for item_data in menu_items_data:
            # Insert menu item using raw SQL
            result = db.execute(
                text("""
                INSERT INTO menu_items (name, description, price, calories, food_category)
                VALUES (:name, :description, :price, :calories, :food_category)
                """),
                {
                    "name": item_data["name"],
                    "description": item_data["description"],
                    "price": item_data["price"],
                    "calories": item_data["calories"],
                    "food_category": item_data["food_category"]
                }
            )
            menu_item_id = result.lastrowid
            
            # Add ingredients
            for ingredient_name, required_amount in item_data["ingredients"]:
                # Check if ingredient exists
                ingredient_result = db.execute(
                    text("SELECT id FROM ingredients WHERE name = :name"),
                    {"name": ingredient_name}
                ).fetchone()
                
                if ingredient_result:
                    ingredient_id = ingredient_result[0]
                else:
                    # Create ingredient with apriltag_id (null for now, can be set later)
                    ing_result = db.execute(
                        text("""
                        INSERT INTO ingredients (restaurant_id, name, unit, quantity_on_hand, reorder_point, apriltag_id)
                        VALUES (:restaurant_id, :name, :unit, :quantity, :reorder_point, :apriltag_id)
                        """),
                        {"restaurant_id": 1, "name": ingredient_name, "unit": "unit", "quantity": 100, "reorder_point": 10, "apriltag_id": None}
                    )
                    ingredient_id = ing_result.lastrowid
                
                # Create recipe relationship
                db.execute(
                    text("""
                    INSERT INTO recipes (menu_item_id, ingredient_id, amount)
                    VALUES (:menu_item_id, :ingredient_id, :amount)
                    """),
                    {
                        "menu_item_id": menu_item_id,
                        "ingredient_id": ingredient_id,
                        "amount": required_amount
                    }
                )
            
            created_items.append({
                "id": menu_item_id,
                "name": item_data["name"],
                "price": item_data["price"],
                "ingredients": len(item_data["ingredients"])
            })
        
        db.commit()
        print(f"Successfully created {len(created_items)} test food items")
        for item in created_items:
            print(f"  - {item['name']}: ${item['price']} ({item['ingredients']} ingredients)")
        return True
        
    except Exception as e:
        db.rollback()
        print(f"Error adding test food: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        db.close()


if __name__ == "__main__":
    add_test_food()
