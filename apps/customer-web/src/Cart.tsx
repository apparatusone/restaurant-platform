import React from "react";
import CartItem from './CartItem';
import { useCart } from './CartContext';
import { Button } from "./components/ui/button";
import { MdClose } from "react-icons/md";

interface CartProps {
    onCheckout: () => void;
}

const Cart: React.FC<CartProps> = ({ onCheckout }) => {
    const { cartItems, totalPrice, isEmpty } = useCart();


    if (isEmpty) {
        return (
            <div className='w-96 flex flex-col justify-start'>
                <div className='flex justify-between'>
                    <div className='text-5xl'>Cart</div>
                    <button className='text-4xl'><MdClose /></button>
                </div>
                <div className='text-center py-8 text-gray-500'>
                    Your cart is empty
                </div>
            </div>
        );
    }

    return (
        <div className='w-96 flex flex-col justify-start'>
            <div className='flex justify-between'>
                <div className='text-5xl'>Cart</div>
                <button className='text-4xl'><MdClose /></button>
            </div>
            <div>
                {cartItems.map((item) => (
                    <CartItem
                        key={item.id}
                        item={item}
                    />
                ))}
            </div>
            <div className='mt-2 text-left'>Promo Code</div>
            <div className='flex items-center gap-2 mb-6'>
                <input type="text" className="border rounded px-2 py-1" placeholder="Enter code" />
                <Button>Apply</Button>
            </div>
            <div className='flex justify-between'>
                <div className="text-2xl font-bold">Total</div>
                <div className="text-lg">${totalPrice.toFixed(2)}</div>
            </div>
        </div>
    );
};

export default Cart;