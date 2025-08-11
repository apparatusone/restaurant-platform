import React from "react";
import CartItem from './CartItem';

interface CartProps {
    onCheckout: () => void;
}

const Cart: React.FC<CartProps> = () => {

    return (
        <div className='w-96 flex flex-col justify-start'>
                    <CartItem
                        key={item.id}
                        item={item}
                    />
        </div>
    );
};

export default Cart;