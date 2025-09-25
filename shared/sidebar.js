// Shared Sidebar Functionality for RoboGPT Documentation
// This script provides consistent sidebar dropdown functionality across all pages

// Initialize sidebar dropdown functionality
function initSidebarDropdowns() {
    const sidebarHeaders = document.querySelectorAll('.sidebar-section h3');
    
    sidebarHeaders.forEach(header => {
        header.addEventListener('click', function() {
            const section = this.parentElement;
            const isCollapsed = section.classList.contains('collapsed');
            
            // Toggle collapsed state
            if (isCollapsed) {
                section.classList.remove('collapsed');
                this.classList.remove('collapsed');
            } else {
                section.classList.add('collapsed');
                this.classList.add('collapsed');
            }
            
            // Save state to localStorage
            saveSidebarState();
        });
    });
}

// Save sidebar collapse state to localStorage
function saveSidebarState() {
    const collapsedSections = [];
    document.querySelectorAll('.sidebar-section.collapsed').forEach(section => {
        const headerText = section.querySelector('h3').textContent.trim();
        collapsedSections.push(headerText);
    });
    localStorage.setItem('sidebarCollapsedSections', JSON.stringify(collapsedSections));
}

// Load sidebar collapse state from localStorage
function loadSidebarState() {
    const savedState = localStorage.getItem('sidebarCollapsedSections');
    if (savedState) {
        const collapsedSections = JSON.parse(savedState);
        collapsedSections.forEach(sectionName => {
            const header = Array.from(document.querySelectorAll('.sidebar-section h3'))
                .find(h => h.textContent.trim() === sectionName);
            if (header) {
                const section = header.parentElement;
                section.classList.add('collapsed');
                header.classList.add('collapsed');
            }
        });
    }
}

// Generate sidebar HTML - can be used to inject sidebar into pages
function generateSidebarHTML(currentPage = '') {
    return `
        <div class="sidebar">
            <div class="sidebar-section">
                <h3>Getting Started</h3>
                <ul>
                    <li><a href="../getting-started/overview.html" ${currentPage === 'overview' ? 'class="active"' : ''}>Overview</a></li>
                    <li><a href="../getting-started/installation.html" ${currentPage === 'installation' ? 'class="active"' : ''}>Installation</a></li>
                    <li><a href="../getting-started/quick-start.html" ${currentPage === 'quick-start' ? 'class="active"' : ''}>Quick Start</a></li>
                    <li><a href="../getting-started/requirements.html" ${currentPage === 'requirements' ? 'class="active"' : ''}>Requirements</a></li>
                </ul>
            </div>

            <div class="sidebar-section">
                <h3>Dev-Docs</h3>
                <ul>
                    <li><a href="../dev-docs/robogpt-agents.html" ${currentPage === 'robogpt-agents' ? 'class="active"' : ''}>Robogpt Agents</a></li>
                    <li><a href="../dev-docs/robogpt-perception.html" ${currentPage === 'robogpt-perception' ? 'class="active"' : ''}>Robogpt Perception</a></li>
                    <li><a href="../dev-docs/robogpt-tools.html" ${currentPage === 'robogpt-tools' ? 'class="active"' : ''}>Robogpt Tools</a></li>
                    <li><a href="../dev-docs/robogpt-nodes.html" ${currentPage === 'robogpt-nodes' ? 'class="active"' : ''}>Robogpt Nodes</a></li>
                    <li><a href="../dev-docs/process-manager.html" ${currentPage === 'process-manager' ? 'class="active"' : ''}>Process Manager</a></li>
                    <li><a href="../dev-docs/prompt-validator.html" ${currentPage === 'prompt-validator' ? 'class="active"' : ''}>Prompt Validator</a></li>
                    <li><a href="../dev-docs/agentic-layer.html" ${currentPage === 'agentic-layer' ? 'class="active"' : ''}>Robogpt Agentic Layer</a></li>
                </ul>
            </div>

            <div class="sidebar-section">
                <h3>Development</h3>
                <ul>
                    <li><a href="../development/contributing.html" ${currentPage === 'contributing' ? 'class="active"' : ''}>Contributing</a></li>
                    <li><a href="../development/testing.html" ${currentPage === 'testing' ? 'class="active"' : ''}>Testing</a></li>
                    <li><a href="../development/troubleshooting.html" ${currentPage === 'troubleshooting' ? 'class="active"' : ''}>Troubleshooting</a></li>
                </ul>
            </div>

            <div class="sidebar-section">
                <h3>Resources</h3>
                <ul>
                    <li><a href="../resources/changelog.html" ${currentPage === 'changelog' ? 'class="active"' : ''}>Changelog</a></li>
                    <li><a href="../resources/faq.html" ${currentPage === 'faq' ? 'class="active"' : ''}>FAQ</a></li>
                    <li><a href="../resources/support.html" ${currentPage === 'support' ? 'class="active"' : ''}>Support</a></li>
                </ul>
            </div>
        </div>
    `;
}

// Initialize everything when DOM is loaded
document.addEventListener('DOMContentLoaded', function() {
    // Initialize dropdown functionality if sidebar exists
    if (document.querySelector('.sidebar')) {
        initSidebarDropdowns();
        loadSidebarState();
    }
});

// Expose functions globally
window.sidebarUtils = {
    initSidebarDropdowns,
    saveSidebarState,
    loadSidebarState,
    generateSidebarHTML
};