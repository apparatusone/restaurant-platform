import json
from pathlib import Path


def load_config():
    """Load restaurant configuration from JSON file"""
    config_path = Path(__file__).parent / "restaurant_config.json"
    with open(config_path, 'r') as f:
        return json.load(f)


# Load config once at module level
CONFIG = load_config()

# Direct access to commonly used values
RESTAURANT_NAME = CONFIG["restaurant"]["name"]
TAX_RATE = CONFIG["business_settings"]["tax_rate"]
CURRENCY_SYMBOL = CONFIG["business_settings"]["currency_symbol"]
PHONE = CONFIG["contact"]["phone"]
EMAIL = CONFIG["contact"]["email"]

# Address as formatted string
ADDRESS = f"{CONFIG['contact']['address']['street']}, {CONFIG['contact']['address']['city']}, {CONFIG['contact']['address']['state']} {CONFIG['contact']['address']['zip_code']}"


def get_config(section=None):
    """Get config section or entire config"""
    if section:
        return CONFIG.get(section, {})
    return CONFIG