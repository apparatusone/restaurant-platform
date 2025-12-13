# api/routers/config.py
from fastapi import APIRouter
from ..config.restaurant import get_config

router = APIRouter(prefix="/config", tags=["config"])

@router.get("/restaurant")
def get_restaurant_config():
    """
    Get restaurant configuration including tax rates, fees, and other settings
    """
    config = get_config()
    
    return {
        "restaurant": {
            "name": config["restaurant"]["name"],
            "phone": config["contact"]["phone"],
            "email": config["contact"]["email"],
            "address": {
                "street": config["contact"]["address"]["street"],
                "city": config["contact"]["address"]["city"],
                "state": config["contact"]["address"]["state"],
                "zip_code": config["contact"]["address"]["zip_code"]
            }
        },
        "pricing": {
            "tax_rate": config["business_settings"]["tax_rate"],
            "service_charge_rate": config["business_settings"]["service_charge_rate"],
            "currency_symbol": config["business_settings"]["currency_symbol"],
            "delivery_fee": config["business_settings"].get("delivery_fee", 3.99),
            "minimum_order": config["business_settings"].get("minimum_order", 15.00)
        },
        "features": {
            "delivery_enabled": config["business_settings"].get("delivery_enabled", True),
            "takeout_enabled": config["business_settings"].get("takeout_enabled", True),
            "dine_in_enabled": config["business_settings"].get("dine_in_enabled", True)
        }
    }

@router.put("/restaurant/pricing")
def update_pricing_config(pricing_data: dict):
    """
    Update pricing configuration (admin only)
    TODO: Add authentication/authorization
    """
    import json
    from pathlib import Path
    
    # Load current config
    config_path = Path(__file__).parent.parent / "config" / "restaurant_config.json"
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    # Update pricing settings
    if "tax_rate" in pricing_data:
        config["business_settings"]["tax_rate"] = float(pricing_data["tax_rate"])
    if "delivery_fee" in pricing_data:
        config["business_settings"]["delivery_fee"] = float(pricing_data["delivery_fee"])
    if "minimum_order" in pricing_data:
        config["business_settings"]["minimum_order"] = float(pricing_data["minimum_order"])
    if "service_charge_rate" in pricing_data:
        config["business_settings"]["service_charge_rate"] = float(pricing_data["service_charge_rate"])
    
    # Save updated config
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=4)
    
    return {"message": "Pricing configuration updated successfully"}