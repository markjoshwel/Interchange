/**
 * Tagcheck Web Application
 * Client-side tag validation and graph visualization
 * 
 * This version uses the bundled library from /dist/index.js
 */

// Global state
let tagRegistry = null;
let currentGraph = null;

// We'll load the validator module dynamically
let TagValidator = null;

// DOM Elements (initialized after DOM load)
let tagInput, validateBtn, resultDiv, resultStatus, resultDetails, graphSvg, exampleBtns, breadcrumbsDiv, autocompleteList;

/**
 * Initialize the application
 */
async function init() {
  // Get DOM elements
  tagInput = document.getElementById('tag-input');
  validateBtn = document.getElementById('validate-btn');
  resultDiv = document.getElementById('result');
  resultStatus = document.getElementById('result-status');
  resultDetails = document.getElementById('result-details');
  graphSvg = document.getElementById('graph-svg');
  exampleBtns = document.querySelectorAll('.example-btn');
  breadcrumbsDiv = document.getElementById('breadcrumbs');
  autocompleteList = document.getElementById('autocomplete-list');
  
  try {
    // Load the bundled validator module
    TagValidator = await import('/dist/index.js');
    
    // Load tags.toml data
    const tagsData = await fetch('/api/tags').then(r => r.json());
    tagRegistry = TagValidator.buildRegistry(tagsData);
    console.log('Registry loaded:', tagRegistry.tags.size, 'tags');
    
    // Setup event listeners
    setupEventListeners();
    
    // Setup autocomplete
    setupAutocomplete();
    
    // Show initial graph with a sample tag
    showTagGraph('Python');
  } catch (error) {
    console.error('Failed to initialize:', error);
    showFallbackMode();
  }
}

/**
 * Fallback mode when API is not available
 */
function showFallbackMode() {
  resultDiv.classList.remove('hidden');
  resultDiv.classList.add('invalid');
  resultStatus.textContent = 'Demo Mode';
  resultDetails.innerHTML = `
    <p>Running in demo mode. Start the dev server to enable full validation:</p>
    <code>bun run dev</code>
  `;
}

/**
 * Setup event listeners
 */
function setupEventListeners() {
  // Validate button click
  validateBtn.addEventListener('click', handleValidate);
  
  // Enter key in input
  tagInput.addEventListener('keypress', (e) => {
    if (e.key === 'Enter') {
      handleValidate();
      // Hide autocomplete on Enter
      if (autocompleteList) autocompleteList.classList.add('hidden');
    }
  });
  
  // Example buttons
  exampleBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      const tag = btn.dataset.tag;
      tagInput.value = tag;
      handleValidate();
    });
  });
  
  // Close dropdowns on outside click
  document.addEventListener('click', (e) => {
    if (!e.target.closest('.input-wrapper')) {
      autocompleteList?.classList.add('hidden');
    }
  });
}

/**
 * Setup autocomplete for the search box
 */
function setupAutocomplete() {
  if (!tagInput || !autocompleteList) return;
  
  tagInput.addEventListener('input', () => {
    const val = tagInput.value;
    if (val.length === 0) {
      autocompleteList.classList.add('hidden');
      return;
    }
    
    // Get last segment if typing a path
    // We want to autocomplete the current tag being typed
    // Logic: Split by / and take the last part
    // But for now, let's keep it simple: global search for the typed text
    // The user requirement implies full search ("swift" -> "Swift")
    
    // Pass high limit to show "everything"
    const results = TagValidator.searchTags(tagRegistry, val, 1000);
    
    if (results.length === 0) {
      autocompleteList.classList.add('hidden');
      return;
    }
    
    autocompleteList.innerHTML = '';
    autocompleteList.classList.remove('hidden');
    
    // Show all results (up to the limit we fetched)
    results.forEach(match => {
      const div = document.createElement('div');
      div.className = 'autocomplete-item';
      
      // Highlight matching part
      const regex = new RegExp(`(${val})`, 'gi');
      const highlighted = match.replace(regex, '<strong>$1</strong>');
      div.innerHTML = highlighted;
      
      div.addEventListener('click', () => {
        // Tag escape if needed
        const escaped = TagValidator.escapeTagName(match);
        tagInput.value = escaped;
        autocompleteList.classList.add('hidden');
        handleValidate();
      });
      
      autocompleteList.appendChild(div);
    });
  });
}

/**
 * Handle tag validation
 */
function handleValidate() {
  const tagString = tagInput.value.trim();
  if (!tagString) {
    resultDiv.classList.add('hidden');
    if (breadcrumbsDiv) breadcrumbsDiv.innerHTML = '';
    return;
  }
  
  if (!tagRegistry || !TagValidator) {
    // Fallback: show demo mode
    showFallbackMode();
    return;
  }
  
  try {
    // Try validation directly first
    let result = TagValidator.validateTag(tagRegistry, tagString);
    
    // If invalid, try fuzzy finding the main tag
    if (!result.valid) {
      const parsed = TagValidator.parseTag(tagString);
      const fuzzyMatch = TagValidator.findTag(tagRegistry, parsed.mainTag);
      
      if (fuzzyMatch && fuzzyMatch !== parsed.mainTag) {
        // Found a better casing or match, try validating with that
        // Reconstruct the tag string with the corrected main tag
        // Note: this only corrects the final tag segment, assuming backlinks are correct-ish
        // or we just validate the found tag directly if it was a fuzzy search for a single term
        
        if (parsed.backlinks.length === 0) {
          // Simple case: just replace the input
          const corrected = TagValidator.escapeTagName(fuzzyMatch);
          // Only update input if it's a simple search
          // tagInput.value = corrected; 
          result = TagValidator.validateTag(tagRegistry, corrected);
        }
      }
    }
    
    showValidationResult(result);
    
    // Update graph and breadcrumbs
    if (result.parsed) {
      showTagGraph(result.parsed.mainTag);
      showBreadcrumbs(result.parsed.mainTag);
    }
  } catch (error) {
    showError(error.message);
  }
}

/**
 * Handle clicking on a graph node - escape the tag name properly
 */
function handleNodeClick(tagName) {
  // Escape the tag name for the parser
  const escapedTag = TagValidator.escapeTagName(tagName);
  tagInput.value = escapedTag;
  handleValidate();
}

/**
 * Show validation result
 */
function showValidationResult(result) {
  resultDiv.classList.remove('hidden');
  
  if (result.valid) {
    resultDiv.classList.remove('invalid');
    resultDiv.classList.add('valid');
    resultStatus.textContent = 'Valid Tag';
    
    let details = `<p><span class="label">Main Tag:</span> <code>${escapeHtml(result.parsed.mainTag)}</code></p>`;
    
    if (result.category) {
      details += `<p><span class="label">Category:</span> <code>${escapeHtml(result.category)}</code></p>`;
    }
    
    if (result.validatedBy && result.validatedBy !== result.parsed.mainTag) {
      details += `<p><span class="label">Validated by:</span> <code>${escapeHtml(result.validatedBy)}</code></p>`;
    }
    
    if (result.parsed.aliases.length > 0) {
      details += `<p><span class="label">Aliases:</span> ${result.parsed.aliases.map(a => `<code>${escapeHtml(a)}</code>`).join(', ')}</p>`;
    }
    
    if (result.parsed.backlinks && result.parsed.backlinks.length > 0) {
      const backlinksStr = result.parsed.backlinks.map(bl => 
        bl.isUnion ? `(${bl.tags.map(escapeHtml).join(', ')})` : escapeHtml(bl.tags[0])
      ).join(' → ');
      details += `<p><span class="label">Backlinks:</span> ${backlinksStr}</p>`;
    }
    
    resultDetails.innerHTML = details;
  } else {
    resultDiv.classList.remove('valid');
    resultDiv.classList.add('invalid');
    resultStatus.textContent = 'Invalid Tag';
    resultDetails.innerHTML = `<p>${escapeHtml(result.error)}</p>`;
  }
}

/**
 * Show breadcrumb paths for a tag
 */
function showBreadcrumbs(tagName) {
  if (!breadcrumbsDiv || !TagValidator || !tagRegistry) return;
  
  const paths = TagValidator.getBreadcrumbPaths(tagRegistry, tagName);
  
  if (paths.length === 0) {
    breadcrumbsDiv.innerHTML = '<p class="no-paths">No paths found</p>';
    return;
  }
  
  // Limit to first 10 paths to avoid overwhelming the UI
  const displayPaths = paths.slice(0, 10);
  
  const html = displayPaths.map(path => {
    const pathHtml = path.map((tag, idx) => {
      const isLast = idx === path.length - 1;
      const escapedTag = TagValidator.escapeTagName(tag);
      return `<span class="breadcrumb-tag${isLast ? ' current' : ''}" data-tag="${escapeHtml(escapedTag)}">${escapeHtml(tag)}</span>`;
    }).join('<span class="breadcrumb-sep">›</span>');
    return `<div class="breadcrumb-path">${pathHtml}</div>`;
  }).join('');
  
  const moreCount = paths.length - displayPaths.length;
  const moreHtml = moreCount > 0 ? `<p class="more-paths">...and ${moreCount} more paths</p>` : '';
  
  breadcrumbsDiv.innerHTML = html + moreHtml;
  
  // Add click handlers to breadcrumb tags
  breadcrumbsDiv.querySelectorAll('.breadcrumb-tag').forEach(el => {
    el.addEventListener('click', () => {
      const tag = el.dataset.tag;
      tagInput.value = tag;
      handleValidate();
    });
  });
}

/**
 * Show error message
 */
function showError(message) {
  resultDiv.classList.remove('hidden', 'valid');
  resultDiv.classList.add('invalid');
  resultStatus.textContent = 'Error';
  resultDetails.innerHTML = `<p>${escapeHtml(message)}</p>`;
  if (breadcrumbsDiv) breadcrumbsDiv.innerHTML = '';
}

/**
 * Escape HTML entities
 */
function escapeHtml(str) {
  const div = document.createElement('div');
  div.textContent = str;
  return div.innerHTML;
}

/**
 * Show tag graph using D3.js
 */
function showTagGraph(centerTag) {
  const container = document.getElementById('graph-container');
  const svg = d3.select('#graph-svg');
  
  // Clear existing graph
  svg.selectAll('*').remove();
  
  if (!tagRegistry || !TagValidator) {
    // Show empty state
    svg.append('text')
      .attr('class', 'graph-empty')
      .attr('x', '50%')
      .attr('y', '50%')
      .attr('text-anchor', 'middle')
      .attr('fill', '#606070')
      .text('Start dev server to enable graph visualization');
    return;
  }
  
  // Get subgraph data
  const graphData = TagValidator.getTagSubgraph(tagRegistry, centerTag, 2);
  
  if (graphData.nodes.length === 0) {
    svg.append('text')
      .attr('x', '50%')
      .attr('y', '50%')
      .attr('text-anchor', 'middle')
      .attr('fill', '#606070')
      .text('No graph data for this tag');
    return;
  }
  
  const width = container.clientWidth;
  const height = container.clientHeight;
  
  // Prepare nodes and links for D3
  const nodes = graphData.nodes.map(name => {
    const tag = TagValidator.getTag(tagRegistry, name);
    return {
      id: name,
      type: name === centerTag ? 'center' : 
            (tag?.frontlinks?.includes(centerTag) ? 'backlink' : 'frontlink')
    };
  });
  
  const links = graphData.edges.map(edge => ({
    source: edge.from,
    target: edge.to
  }));
  
  // Create force simulation
  const simulation = d3.forceSimulation(nodes)
    .force('link', d3.forceLink(links).id(d => d.id).distance(100))
    .force('charge', d3.forceManyBody().strength(-300))
    .force('center', d3.forceCenter(width / 2, height / 2))
    .force('collision', d3.forceCollide().radius(40));
  
  // Add arrow marker
  svg.append('defs').append('marker')
    .attr('id', 'arrowhead')
    .attr('viewBox', '-0 -5 10 10')
    .attr('refX', 20)
    .attr('refY', 0)
    .attr('orient', 'auto')
    .attr('markerWidth', 6)
    .attr('markerHeight', 6)
    .append('path')
    .attr('d', 'M 0,-5 L 10,0 L 0,5')
    .attr('fill', 'rgba(160, 160, 176, 0.5)');
  
  // Create links
  const link = svg.append('g')
    .selectAll('line')
    .data(links)
    .enter().append('line')
    .attr('class', 'link')
    .attr('marker-end', 'url(#arrowhead)');
  
  // Create nodes
  const node = svg.append('g')
    .selectAll('g')
    .data(nodes)
    .enter().append('g')
    .attr('class', 'node')
    .style('cursor', 'pointer')
    .on('click', (event, d) => {
      // Use the handleNodeClick function to properly escape tag names
      handleNodeClick(d.id);
    })
    .call(d3.drag()
      .on('start', dragstarted)
      .on('drag', dragged)
      .on('end', dragended));
  
  // Add circles to nodes
  node.append('circle')
    .attr('r', d => d.type === 'center' ? 14 : 10)
    .attr('fill', d => {
      switch (d.type) {
        case 'center': return '#7c3aed';
        case 'backlink': return '#06b6d4';
        case 'frontlink': return '#10b981';
        default: return '#606070';
      }
    })
    .style('filter', d => d.type === 'center' ? 'drop-shadow(0 0 8px #7c3aed)' : 'none');
  
  // Add labels
  node.append('text')
    .attr('dy', -18)
    .attr('text-anchor', 'middle')
    .text(d => d.id.length > 20 ? d.id.substring(0, 18) + '...' : d.id);
  
  // Update positions on tick
  simulation.on('tick', () => {
    link
      .attr('x1', d => d.source.x)
      .attr('y1', d => d.source.y)
      .attr('x2', d => d.target.x)
      .attr('y2', d => d.target.y);
    
    node.attr('transform', d => `translate(${d.x},${d.y})`);
  });
  
  // Drag functions
  function dragstarted(event, d) {
    if (!event.active) simulation.alphaTarget(0.3).restart();
    d.fx = d.x;
    d.fy = d.y;
  }
  
  function dragged(event, d) {
    d.fx = event.x;
    d.fy = event.y;
  }
  
  function dragended(event, d) {
    if (!event.active) simulation.alphaTarget(0);
    d.fx = null;
    d.fy = null;
  }
  
  // Store current simulation for cleanup
  if (currentGraph) {
    currentGraph.stop();
  }
  currentGraph = simulation;
}

// Handle window resize
let resizeTimeout;
window.addEventListener('resize', () => {
  clearTimeout(resizeTimeout);
  resizeTimeout = setTimeout(() => {
    const currentValue = tagInput?.value?.trim();
    if (currentValue && tagRegistry && TagValidator) {
      try {
        const parsed = TagValidator.parseTag(currentValue);
        showTagGraph(parsed.mainTag);
      } catch (e) {
        // Ignore parse errors on resize
      }
    }
  }, 250);
});

// Initialize on DOM ready
document.addEventListener('DOMContentLoaded', init);
