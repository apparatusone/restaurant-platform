import { useState } from 'react'
import Menu from './Menu'
import Cart from './Cart'
import Checkout from './pages/Checkout'
import { Toaster } from "@/components/ui/sonner"
import Header from './Header'
import { CartProvider } from './CartContext'
import './App.css'

// interface {

// }

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
      <div>
        <a href="https://vite.dev" target="_blank">
          <img src={viteLogo} className="logo" alt="Vite logo" />
        </a>
        <a href="https://react.dev" target="_blank">
          <img src={reactLogo} className="logo react" alt="React logo" />
        </a>
      </div>
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
