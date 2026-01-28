from sqlalchemy.orm import Session
from sqlalchemy import func
from fastapi import HTTPException, status, Response
from sqlalchemy.exc import SQLAlchemyError
from datetime import datetime, date
from ..models.recipes import Recipe
from ..models.ingredients import Ingredient
from ..models.payment_method import Payment, PaymentStatus
from shared.models.menu_items import MenuItem
from shared.utils.error_handlers import handle_database_error
    

# this function gets and returns the ingredients needed for a particular menu item
def get_required_ingredients(db, menu_item_id, quantity):
    ingredients = db.query(Recipe).join(Ingredient).filter(
        Recipe.menu_item_id == menu_item_id
    ).all()

    return {
        ingredient.ingredient.name: ingredient.amount * quantity
        for ingredient in ingredients
    }

def get_daily_revenue(db: Session, target_date: date):
    """
    Get total revenue and completed orders for a specific date (YYYY-MM-DD format)
    """
    from datetime import datetime, timedelta
    from ..models.checks import Check
    
    start_of_day = datetime.combine(target_date, datetime.min.time())
    end_of_day = datetime.combine(target_date, datetime.max.time())
    
    completed_checks = db.query(Check).join(
        Payment, Check.id == Payment.check_id
    ).filter(
        Check.opened_at >= start_of_day
    ).filter(
        Check.opened_at <= end_of_day
    ).filter(
        Payment.status == PaymentStatus.COMPLETED
    ).all()
    
    # Calculate total revenue using stored final_total
    total_revenue = sum(float(check.final_total) for check in completed_checks if check.final_total)
    check_count = len(completed_checks)
    
    return {
        "date": target_date.isoformat(),
        "completed_orders": check_count,
        "total_revenue": round(total_revenue, 2)
    }
    

def add_menu_item(db: Session, request):
    """
    Create a new menu item with ingredients and add empty ingredients
    """
    from ..controllers import menu_items as menu_item_controller
    from ..schemas.menu_items import MenuItemsCreate
    from ..models.recipes import Recipe
    from ..models.ingredients import Ingredient
    
    try:
        # create the menu item
        menu_item_data = MenuItemsCreate(
            name=request.name,
            description=request.description,
            price=request.price,
            calories=request.calories,
            food_category=request.food_category
        )
        
        new_menu_item = menu_item_controller.create(db=db, request=menu_item_data)
        
        # process each ingredient needed and track new ones
        new_ingredients_added = []
        
        for ingredient_req in request.resources:
            # check if ingredient exists by name, create if it doesn't
            existing_ingredient = db.query(Ingredient).filter(Ingredient.name == ingredient_req.resource_name).first()
            if not existing_ingredient:
                # create new ingredient with quantity_on_hand = 0
                new_ingredient = Ingredient(
                    name=ingredient_req.resource_name,
                    quantity_on_hand=0
                )
                db.add(new_ingredient)
                db.flush()  # Get the ID
                ingredient_id = new_ingredient.id
                new_ingredients_added.append(ingredient_req.resource_name)
            else:
                ingredient_id = existing_ingredient.id
            
            # create recipe relationship
            recipe = Recipe(
                menu_item_id=new_menu_item.id,
                ingredient_id=ingredient_id,
                amount=ingredient_req.quantity
            )
            db.add(recipe)
        
        db.commit()
        
        return {
            "menu_item": new_menu_item.name,
            "resources_added": new_ingredients_added # only includes new ingredients
        }
        
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Failed to create menu item: {str(e)}")


def update_stock(db: Session, resource_name: str, amount_change: int):
    """
    Add or remove stock for an ingredient
    """
    from ..models.ingredients import Ingredient
    
    try:
        # find the ingredient by name
        ingredient = db.query(Ingredient).filter(Ingredient.name == resource_name).first()
        
        if not ingredient:
            raise HTTPException(status_code=404, detail=f"Ingredient '{resource_name}' not found")
        
        # handle zero change
        if amount_change == 0:
            return {
                "message": f"No change made to {resource_name} stock"
            }
        
        # calc the new stock amount
        new_amount = ingredient.quantity_on_hand + amount_change
        
        # prevent negative stock
        if new_amount < 0:
            raise HTTPException(
                status_code=400, 
                detail=f"Cannot reduce stock below 0. Current: {ingredient.quantity_on_hand}, Requested change: {amount_change}"
            )
        
        # update the stock quantity
        ingredient.quantity_on_hand = new_amount
        db.commit()
        
        return {
            "previous_amount": ingredient.quantity_on_hand - amount_change,
            "new_amount": ingredient.quantity_on_hand,
            "message": f"Stock updated for {resource_name}"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Failed to update stock: {str(e)}")
