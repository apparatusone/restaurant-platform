import { useCart } from "./CartContext";
import type { LocalCartItem } from "./models";
import { LuPlus, LuMinus } from "react-icons/lu";
interface CartItemProps {
    item: LocalCartItem;
}
const CartItem: React.FC<CartItemProps> = ({ item }) => {

    return (
        <div className="border-b px-2 pt-4 pb-2 flex flex-col justify-start gap-4">
            <div className="flex justify-between">
                <div className="font-semibold">{item.name}</div>

                {/* quantity x price */}
                <div className="font-semibold">${(item.quantity * item.price).toFixed(2)}</div>
            </div>
            {/* <div className="text-sm text-gray-600 mb-2 max-w-[80%] text-left">{item.description}</div> */}
            <div className="flex justify-between gap-4">
                <button className="text-blue-600 hover:underline text-sm" onClick={removeItem}>Remove</button>
                <div className="flex justify-between border border-b-gray-300 rounded-xl">
                    <button className="w-8 h-8 flex items-center justify-center p-1" 
                            <LuPlus className="inline" />
                    </button>
                    <span className="w-6 h-8 flex items-center justify-center">{item.quantity}</span>
                    <button className="w-8 flex items-center justify-center text-xl p-0" 
                            <LuMinus className="inline" />
                    </button>
                </div>
            </div>
        </div>
    );
};

export default CartItem;
