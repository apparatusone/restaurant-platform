from api.dependencies.database import SessionLocal
from api.models.customers import Customer
from api.models import model_loader


def add_test_customers():
    model_loader.index()
    
    db = SessionLocal()
    
    try:
        customers_data = [
            {
                "customer_name": "Jason Bourne",
                "customer_email": "jason.bourne@email.com",
                "customer_phone": 1234567,
                "customer_address": "Blue Ridge Parkway, Asheville, NC 28801"
            },
            {
                "customer_name": "One Punch Man",
                "customer_email": "one.punch.man@email.com", 
                "customer_phone": 9876543,
                "customer_address": "Wright Brothers Memorial, Kill Devil Hills, NC 27948"
            },
            {
                "customer_name": "Peter Peterson",
                "customer_email": "peter.peterson@email.com",
                "customer_phone": 5551234,
                "customer_address": "Cape Hatteras Lighthouse, Buxton, NC 27920"
            }
        ]
        
        created_customers = []
        
        for customer_data in customers_data:
            customer = Customer(
                customer_name=customer_data["customer_name"],
                customer_email=customer_data["customer_email"],
                customer_phone=customer_data["customer_phone"],
                customer_address=customer_data["customer_address"]
            )
            
            db.add(customer)
            db.flush()
            
            created_customers.append({
                "id": customer.id,
                "name": customer.customer_name,
                "email": customer.customer_email,
                "phone": customer.customer_phone
            })
        
        db.commit()
        return True
        
    except Exception as e:
        db.rollback()
        print(f"Error adding test customers: {e}")
        return False
    
    finally:
        db.close()


if __name__ == "__main__":
    add_test_customers()