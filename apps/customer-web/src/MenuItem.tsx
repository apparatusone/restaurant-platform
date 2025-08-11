import React from "react";
import { toast } from "sonner"
import { useCart } from "./CartContext";
import type { MenuItem as MenuItemType } from "./models";
import { Card } from "./components/ui/card";
import { Button } from "./components/ui/button";
import { IoMdInformationCircleOutline } from "react-icons/io";
import { IoCloseCircleSharp } from "react-icons/io5";

interface MenuItemProps {
    item: MenuItemType;
}

const MenuItem: React.FC<MenuItemProps> = ({ item }) => {
    const { addItem } = useCart();

    function addToCart() {
        try {
            addItem(item, 1);
            toast.success(`${item.name} added to cart`);
        } catch (err) {
            console.error('Add to cart error:', err);
            toast.error('Failed to add to cart');
        }
    }

    const [showDescription, setShowDescription] = React.useState(false);

    return (
        <Card className="w-56 h-40 p-2 flex items-center gap-3 relative overflow-hidden">
            {/* Main content always in flex row, never hidden */}
            <button
                className="ml-auto"
                aria-label={`Show description for ${item.name}`}
                title={`Show description for ${item.name}`}
                onClick={() => setShowDescription(true)}
            >
                <IoMdInformationCircleOutline size="1.3rem" />
            </button>
            <p>{item.name}</p>
            <p>${item.price}</p>
            <Button
                className="w-[70%]"
                onClick={() => addToCart()}
            >
                Add To Cart
            </Button>

            <div
                className={
                    "absolute inset-0 flex flex-col items-center justify-center bg-white z-10 transition-transform duration-200 " +
                    (showDescription ? "translate-x-0" : "translate-x-full")
                }
                aria-live="polite"
            >
                <button
                    className="absolute top-2 right-2"
                    aria-label="Close description"
                    title="Close description"
                    onClick={() => setShowDescription(false)}
                >
                    <IoCloseCircleSharp size="1.5rem" />
                </button>
                <p className="text-center px-4 text-sm">{item.description}</p>
            </div>
        </Card>
    );
};

export default MenuItem;
