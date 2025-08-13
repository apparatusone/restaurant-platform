import React from "react";
import { FaGithub } from "react-icons/fa";

const Footer: React.FC = () => {
    return (
        <footer className="w-full bg-gray-100 border-t border-gray-200 mt-8">
            <div className="max-w-6xl mx-auto px-4 py-6">
                <div className="flex flex-col md:flex-row justify-between items-center gap-4">
                    {/* Left side - Project info */}
                    <div className="text-center md:text-left">
                        <h3 className="font-semibold text-gray-800">Pizza² Demo</h3>
                        <p className="text-sm text-gray-600">
                            Full-stack restaurant ordering system demo
                        </p>
                    </div>

                    {/* Center - Tech stack */}
                    <div className="text-center">
                        <p className="text-xs text-gray-500 mb-1">Built with</p>
                        <div className="flex flex-wrap justify-center gap-2 text-xs text-gray-600">
                            <span className="bg-blue-100 px-2 py-1 rounded">React</span>
                            <span className="bg-green-100 px-2 py-1 rounded">TypeScript</span>
                            <span className="bg-purple-100 px-2 py-1 rounded">FastAPI</span>
                            <span className="bg-yellow-100 px-2 py-1 rounded">Python</span>
                        </div>
                    </div>

                    {/* Right side - Links */}
                    <div className="flex items-center gap-4">
                        <a
                            href="https://github.com/your-username/KitchenOps-Demo"
                            target="_blank"
                            rel="noopener noreferrer"
                            className="flex items-center gap-2 text-gray-600 hover:text-gray-800 transition-colors"
                            title="View source code on GitHub"
                        >
                            <FaGithub size="1.2rem" />
                            <span className="text-sm">Source Code</span>
                        </a>
                    </div>
                </div>

                {/* Bottom row */}
                <div className="mt-4 pt-4 border-t border-gray-200 text-center">
                    <p className="text-xs text-gray-500">
                        Demo project • Not a real restaurant • No actual payments processed
                    </p>
                </div>
            </div>
        </footer>
    );
};

export default Footer;