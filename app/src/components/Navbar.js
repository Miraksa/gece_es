import React from 'react';
import "./Navbar.css";
import "./Ros2connection.js";
import Ros2Connection from './Ros2connection.js';

export default function Navbar() {
    return (
        <div className="Navbar">
            <h1>Gece ES</h1>
            <Ros2Connection />
        </div>
    )
}