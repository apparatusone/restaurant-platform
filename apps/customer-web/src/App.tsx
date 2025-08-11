import { Toaster } from "@/components/ui/sonner"
import { CartProvider } from './CartContext'
import './App.css'

// interface {

// }

function App() {
  const [count, setCount] = useState(0)

    const [menuItems, setMenuItems] = useState([])
    const [loading, setLoading] = useState(true)
    const [error, setError] = useState<string | null>(null)

    useEffect(() => {
        const fetchMenuItems = async () => {
        try {
            const response = await fetch('http://localhost:8000/customer/menu')
            if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`)
            }
            const data = await response.json()
            setMenuItems(data)
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Failed to fetch menu items')
        } finally {
            setLoading(false)
        }
        }

        fetchMenuItems()
    }, [])

    if (loading) return <div>Loading menu items...</div>
    if (error) return <div>Error: {error}</div>

  return (
        <CartProvider>
      <div>
        <a href="https://vite.dev" target="_blank">
          <img src={viteLogo} className="logo" alt="Vite logo" />
        </a>
        <a href="https://react.dev" target="_blank">
          <img src={reactLogo} className="logo react" alt="React logo" />
        </a>
      </div>
            <Toaster richColors position="top-center" />
                    <Cart />
      </div>
      <p className="read-the-docs">
        Click on the Vite and React logos to learn more
      </p>
        </CartProvider>
  )
}

export default App
