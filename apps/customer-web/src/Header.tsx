import React from "react";

const Header: React.FC = () => {
    return (
        <div className="flex items-center w-full h-20 mb-4 bg-blue-200 px-4">
            <img
                src="/pizza2_logo.svg"
                alt="Pizza Pizza Logo"
                className="h-20 w-20 mr-4"
                style={{ objectFit: 'contain' }}
            />
            <div>
                <h1 className="text-2xl font-bold">
                    PIZZA
                    <sup className="text-[1.3rem] align-super ml-0.5">2</sup>
                </h1>
                <h2 className="text-lg">Menu</h2>
            </div>
        </div>
    );
};

export default Header;
