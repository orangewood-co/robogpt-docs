/**
 * Theme Toggle System for RoboGPT Documentation
 * Handles dark/light theme switching with localStorage persistence
 */

class ThemeToggle {
    constructor() {
        this.currentTheme = this.getStoredTheme() || this.getSystemTheme();
        this.init();
    }

    init() {
        this.createToggleButton();
        this.applyTheme(this.currentTheme);
        this.addEventListeners();
    }

    createToggleButton() {
        // Remove existing button if it exists
        const existingButton = document.querySelector('.theme-toggle');
        if (existingButton) {
            existingButton.remove();
        }

        // Create the toggle button
        const toggleButton = document.createElement('button');
        toggleButton.className = 'theme-toggle';
        toggleButton.setAttribute('aria-label', 'Toggle theme');
        toggleButton.setAttribute('title', 'Toggle between light and dark theme');
        
        // Set initial icon
        this.updateButtonIcon(toggleButton);
        
        // Add to document
        document.body.appendChild(toggleButton);
        
        this.toggleButton = toggleButton;
    }

    updateButtonIcon(button) {
        // Use sun icon for light theme, moon icon for dark theme
        if (this.currentTheme === 'dark') {
            button.innerHTML = '☀️'; // Sun icon for switching to light
            button.setAttribute('title', 'Switch to light theme');
        } else {
            button.innerHTML = '🌙'; // Moon icon for switching to dark
            button.setAttribute('title', 'Switch to dark theme');
        }
    }

    addEventListeners() {
        this.toggleButton.addEventListener('click', () => {
            this.toggleTheme();
        });

        // Listen for system theme changes
        window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', (e) => {
            if (!this.getStoredTheme()) {
                this.currentTheme = e.matches ? 'dark' : 'light';
                this.applyTheme(this.currentTheme);
                this.updateButtonIcon(this.toggleButton);
            }
        });

        // Handle keyboard navigation
        this.toggleButton.addEventListener('keydown', (e) => {
            if (e.key === 'Enter' || e.key === ' ') {
                e.preventDefault();
                this.toggleTheme();
            }
        });
    }

    toggleTheme() {
        this.currentTheme = this.currentTheme === 'light' ? 'dark' : 'light';
        this.applyTheme(this.currentTheme);
        this.storeTheme(this.currentTheme);
        this.updateButtonIcon(this.toggleButton);
        
        // Dispatch custom event for other components that might need to know about theme change
        window.dispatchEvent(new CustomEvent('themeChanged', {
            detail: { theme: this.currentTheme }
        }));
    }

    applyTheme(theme) {
        if (theme === 'dark') {
            document.documentElement.setAttribute('data-theme', 'dark');
        } else {
            document.documentElement.removeAttribute('data-theme');
        }
    }

    storeTheme(theme) {
        try {
            localStorage.setItem('robogpt-docs-theme', theme);
        } catch (e) {
            console.warn('Failed to save theme preference:', e);
        }
    }

    getStoredTheme() {
        try {
            return localStorage.getItem('robogpt-docs-theme');
        } catch (e) {
            console.warn('Failed to retrieve theme preference:', e);
            return null;
        }
    }

    getSystemTheme() {
        if (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches) {
            return 'dark';
        }
        return 'light';
    }

    // Public method to get current theme
    getCurrentTheme() {
        return this.currentTheme;
    }

    // Public method to set theme programmatically
    setTheme(theme) {
        if (theme === 'light' || theme === 'dark') {
            this.currentTheme = theme;
            this.applyTheme(this.currentTheme);
            this.storeTheme(this.currentTheme);
            if (this.toggleButton) {
                this.updateButtonIcon(this.toggleButton);
            }
        }
    }
}

// Initialize theme toggle when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    // Create global theme toggle instance
    window.themeToggle = new ThemeToggle();
});

// Also initialize if DOM is already loaded
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
        if (!window.themeToggle) {
            window.themeToggle = new ThemeToggle();
        }
    });
} else {
    // DOM is already ready
    if (!window.themeToggle) {
        window.themeToggle = new ThemeToggle();
    }
}

// Export for module usage if needed
if (typeof module !== 'undefined' && module.exports) {
    module.exports = ThemeToggle;
}