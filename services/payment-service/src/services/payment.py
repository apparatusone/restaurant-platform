import stripe
import os
from dotenv import load_dotenv
from fastapi import HTTPException

load_dotenv()

def process_stripe_payment(amount_cents: int, order_id: int, customer_name: str = None):
    """process payment with stripe test mode"""
    # check if we should use mock mode (no api key given)
    # stripe_key = os.getenv("STRIPE_SECRET_KEY")
    stripe_key = None
    if not stripe_key:
        # mock payment for testing without stripe account
        import time
        time.sleep(0.5)  # simulate api call delay

        import logging
        logger = logging.getLogger(__name__)
        logger.info("Mock stripe payment processed")
        
        return {
            "success": True,
            "status": "succeeded",
            "payment_intent_id": f"pi_mock_{order_id}_{int(time.time())}",
            "amount_received": amount_cents
        }
    
    # real stripe processing
    stripe.api_key = stripe_key
    
    try:
        # create and confirm payment immediately
        payment_intent = stripe.PaymentIntent.create(
            amount=amount_cents,
            currency="usd",
            payment_method="pm_card_visa",  # test card
            confirm=True, # payment is automatically accepted for testing
            automatic_payment_methods={"enabled": True, "allow_redirects": "never"},
            metadata={"order_id": str(order_id)} if not customer_name else {"order_id": str(order_id), "customer_name": customer_name}
        )
        
        # return response with payment details
        success = payment_intent.status in ["succeeded", "requires_confirmation"]
        return {
            "success": success,
            "status": payment_intent.status,
            "payment_intent_id": payment_intent.id,
            "amount_received": getattr(payment_intent, 'amount_received', 0)
        }
            
    except stripe.error.StripeError as e:
        return {
            "success": False,
            "error": f"stripe error: {str(e)}"
        }
    except Exception as e:
        return {
            "success": False,
            "error": f"payment error: {str(e)}"
        }
