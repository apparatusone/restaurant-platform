#!/usr/bin/env python3
"""
Script to add all test food items to the database
"""

from api.dependencies.database import SessionLocal
from api.models.menu_items import MenuItem, FoodCategory
from api.models.resources import Resource
from api.models.menu_item_ingredients import MenuItemIngredient
from api.models import model_loader


# main dishes
# stolen from dominos
MAIN_DISHES = [
    {
        "name": "ExtravaganZZa Pizza",
        "description": "Pepperoni, ham, Italian sausage, beef, onions, green peppers, mushrooms, and black olives",
        "price": 16.99,
        "calories": 320,
        "food_category": FoodCategory.REGULAR,
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
        "name": "MeatZZa Pizza",
        "description": "Pepperoni, ham, beef, Italian sausage, and extra cheese",
        "price": 15.99,
        "calories": 350,
        "food_category": FoodCategory.REGULAR,
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
        "name": "Deluxe Pizza",
        "description": "Pepperoni, Italian sausage, green peppers, mushrooms, and onions",
        "price": 14.99,
        "calories": 290,
        "food_category": FoodCategory.REGULAR,
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
        "food_category": FoodCategory.REGULAR,
        "ingredients": [
            ("Pizza Dough", 1),
            ("Pizza Sauce", 3),
            ("Mozzarella Cheese", 5),
            ("Ham", 4),
            ("Pineapple", 3)
        ]
    }
]

# ==================== SIDE ITEMS ====================
SIDE_ITEMS = [
    {
        "name": "Small Ranch Cup",
        "description": "Small cup of ranch dressing",
        "price": 0.25,
        "calories": 120,
        "food_category": FoodCategory.REGULAR,
        "ingredients": [
            ("Ranch Dressing", 1)
        ]
    },
    {
        "name": "Garden Fresh Salad",
        "description": "Fresh lettuce, tomatoes, red onions, and green peppers",
        "price": 6.99,
        "calories": 70,
        "food_category": FoodCategory.REGULAR,
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
        "food_category": FoodCategory.REGULAR,
        "ingredients": [
            ("Romaine Lettuce", 3),
            ("Grilled Chicken", 4),
            ("Parmesan Cheese", 2),
            ("Croutons", 1),
            ("Caesar Dressing", 2)
        ]
    }
]

# ==================== BEVERAGES ====================
BEVERAGES = [
    {
        "name": "Coke Bottle",
        "description": "12oz Coca-Cola bottle",
        "price": 2.50,
        "calories": 140,
        "food_category": FoodCategory.REGULAR,
        "ingredients": [
            ("Coke Bottle", 1) # direct inventory?
        ]
    },
    {
        "name": "Water Bottle",
        "description": "16oz bottled water",
        "price": 1.99,
        "calories": 0,
        "food_category": FoodCategory.REGULAR,
        "ingredients": [
            ("Water Bottle", 1)
        ]
    }
]

def add_test_food():
    model_loader.index()
    
    db = SessionLocal()
    
    try:
        # combine all menu items
        menu_items_data = MAIN_DISHES + SIDE_ITEMS + BEVERAGES
        
        created_items = []
        
        # add each menu item
        for item_data in menu_items_data:
            menu_item = MenuItem(
                name=item_data["name"],
                description=item_data["description"],
                price=item_data["price"],
                calories=item_data["calories"],
                food_category=item_data["food_category"]
            )
            
            db.add(menu_item)
            db.flush()
            
            # add ingredients
            for ingredient_name, required_amount in item_data["ingredients"]:
                # check if resource exists
                resource = db.query(Resource).filter(Resource.item == ingredient_name).first()
                
                if not resource:
                    # create resource with arbitrary stock
                    initial_stock = 10
                    resource = Resource(
                        item=ingredient_name,
                        amount=initial_stock
                    )
                    db.add(resource)
                    db.flush()
                
                # create relationship
                menu_ingredient = MenuItemIngredient(
                    menu_item_id=menu_item.id,
                    resource_id=resource.id,
                    amount=required_amount
                )
                db.add(menu_ingredient)
            
            created_items.append({
                "id": menu_item.id,
                "name": menu_item.name,
                "price": menu_item.price,
                "calories": menu_item.calories,
                "ingredients": len(item_data["ingredients"])
            })
        
        db.commit()
        print(f"Successfully created {len(created_items)} test food items")
        return True
        
    except Exception as e:
        db.rollback()
        print(f"Error adding test food: {e}")
        return False
    
    finally:
        db.close()


if __name__ == "__main__":
    add_test_food()