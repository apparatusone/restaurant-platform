import { useState, useEffect } from 'react'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
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
    <>
      <div>
        <a href="https://vite.dev" target="_blank">
          <img src={viteLogo} className="logo" alt="Vite logo" />
        </a>
        <a href="https://react.dev" target="_blank">
          <img src={reactLogo} className="logo react" alt="React logo" />
        </a>
      </div>
      <h1>Vite + React</h1>
      <div className="card">
        <button onClick={() => setCount((count) => count + 1)}>
          count is {count}
        </button>
        <p>
          Edit <code>src/App.tsx</code> and save to test HMR
        </p>
      </div>
      <p className="read-the-docs">
        Click on the Vite and React logos to learn more
      </p>
    </>
  )
}

export default App
