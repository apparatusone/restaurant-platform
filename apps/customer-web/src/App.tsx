import { useState } from 'react'
import Menu from './Menu'
import Cart from './Cart'
import Checkout from './pages/Checkout'
import { Toaster } from "@/components/ui/sonner"
import Header from './Header'
import { CartProvider } from './CartContext'
import './App.css'

function App() {
    const [currentView, setCurrentView] = useState<'menu' | 'checkout'>('menu');

    const handleGoToCheckout = () => {
        setCurrentView('checkout');
    };

    const handleBackToMenu = () => {
        setCurrentView('menu');
    };

    return (
        <CartProvider>
            <Header/>
            <Toaster richColors position="top-center" />
            
            {currentView === 'menu' ? (
                <div className='flex'>
                    <Menu />
                    <Cart onCheckout={handleGoToCheckout} />
                </div>
            ) : (
                <Checkout onBack={handleBackToMenu} />
            )}
        </CartProvider>
    )
}

export default App
