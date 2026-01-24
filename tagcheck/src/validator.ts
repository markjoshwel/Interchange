/**
 * Tag validator that checks tags against the tags.toml reference
 */

import { parseTag, getAllMentionedTags } from './parser';
import type { 
  ParsedTag, 
  ResolvedTag, 
  ValidationResult, 
  TagRegistry, 
  TagsToml 
} from './types';

/**
 * Build a tag registry from the parsed tags.toml structure
 */
export function buildRegistry(tagsToml: TagsToml): TagRegistry {
  const tags = new Map<string, ResolvedTag>();
  const aliases = new Map<string, string>();
  const categories: string[] = [];
  
  const referenceData = tagsToml.interchange.reference.tags;
  
  // First pass: collect all categories
  for (const category of Object.keys(referenceData)) {
    categories.push(category);
  }
  
  // Second pass: parse all tags and build initial registry
  for (const [category, tagStrings] of Object.entries(referenceData)) {
    for (const tagString of tagStrings) {
      try {
        const parsed = parseTag(tagString);
        const effectiveCategory = parsed.category || category;
        
        // Get or create the resolved tag for the main tag
        const existing = tags.get(parsed.mainTag);
        if (existing) {
          // Merge with existing
          if (parsed.category && !existing.directCategory) {
            existing.directCategory = parsed.category;
          }
          if (!existing.indirectCategories.includes(category)) {
            existing.indirectCategories.push(category);
          }
          for (const alias of parsed.aliases) {
            if (!existing.aliases.includes(alias)) {
              existing.aliases.push(alias);
            }
          }
        } else {
          // Create new resolved tag
          const resolved: ResolvedTag = {
            name: parsed.mainTag,
            aliases: [...parsed.aliases],
            directCategory: parsed.category,
            indirectCategories: [category],
            backlinks: [],
            frontlinks: [],
          };
          tags.set(parsed.mainTag, resolved);
        }
        
        // Register aliases
        for (const alias of parsed.aliases) {
          aliases.set(alias, parsed.mainTag);
        }
        
        // Process backlinks to establish relationships
        if (parsed.backlinks.length > 0) {
          const mainResolved = tags.get(parsed.mainTag)!;
          const immediateBacklinks = parsed.backlinks[parsed.backlinks.length - 1].tags;
          
          for (const backlinkTag of immediateBacklinks) {
            if (!mainResolved.backlinks.includes(backlinkTag)) {
              mainResolved.backlinks.push(backlinkTag);
            }
            
            // Ensure backlink tag exists and add frontlink
            if (!tags.has(backlinkTag)) {
              tags.set(backlinkTag, {
                name: backlinkTag,
                aliases: [],
                indirectCategories: [],
                backlinks: [],
                frontlinks: [parsed.mainTag],
              });
            } else {
              const backlinkResolved = tags.get(backlinkTag)!;
              if (!backlinkResolved.frontlinks.includes(parsed.mainTag)) {
                backlinkResolved.frontlinks.push(parsed.mainTag);
              }
            }
          }
          
          // Process chain: A / B / C means B <- A, C <- B
          for (let i = 0; i < parsed.backlinks.length - 1; i++) {
            const parentLinks = parsed.backlinks[i].tags;
            const childLinks = parsed.backlinks[i + 1].tags;
            
            for (const parentTag of parentLinks) {
              for (const childTag of childLinks) {
                // Ensure parent exists
                if (!tags.has(parentTag)) {
                  tags.set(parentTag, {
                    name: parentTag,
                    aliases: [],
                    indirectCategories: [],
                    backlinks: [],
                    frontlinks: [],
                  });
                }
                
                // Ensure child exists
                if (!tags.has(childTag)) {
                  tags.set(childTag, {
                    name: childTag,
                    aliases: [],
                    indirectCategories: [],
                    backlinks: [],
                    frontlinks: [],
                  });
                }
                
                const parentResolved = tags.get(parentTag)!;
                const childResolved = tags.get(childTag)!;
                
                if (!parentResolved.frontlinks.includes(childTag)) {
                  parentResolved.frontlinks.push(childTag);
                }
                if (!childResolved.backlinks.includes(parentTag)) {
                  childResolved.backlinks.push(parentTag);
                }
              }
            }
          }
        }
      } catch (e) {
        console.warn(`Failed to parse tag "${tagString}" in category "${category}":`, e);
      }
    }
  }
  
  return { tags, aliases, categories };
}

/**
 * Resolve an alias to its primary tag name
 */
export function resolveAlias(registry: TagRegistry, tagName: string): string {
  return registry.aliases.get(tagName) || tagName;
}

/**
 * Check if a tag exists in the registry (directly or as alias)
 */
export function tagExists(registry: TagRegistry, tagName: string): boolean {
  const resolved = resolveAlias(registry, tagName);
  return registry.tags.has(resolved);
}

/**
 * Get a resolved tag from the registry
 */
export function getTag(registry: TagRegistry, tagName: string): ResolvedTag | undefined {
  const resolved = resolveAlias(registry, tagName);
  return registry.tags.get(resolved);
}

/**
 * Get all backlinks for a tag (recursive)
 */
export function getAllBacklinks(registry: TagRegistry, tagName: string, visited: Set<string> = new Set()): string[] {
  const resolved = resolveAlias(registry, tagName);
  if (visited.has(resolved)) {
    return [];
  }
  visited.add(resolved);
  
  const tag = registry.tags.get(resolved);
  if (!tag) {
    return [];
  }
  
  const result: string[] = [...tag.backlinks];
  for (const backlink of tag.backlinks) {
    result.push(...getAllBacklinks(registry, backlink, visited));
  }
  
  return result;
}

/**
 * Infer the category for a tag based on its backlinks
 */
export function inferCategory(registry: TagRegistry, tagName: string): string | undefined {
  const resolved = resolveAlias(registry, tagName);
  const tag = registry.tags.get(resolved);
  
  if (tag?.directCategory) {
    return tag.directCategory;
  }
  
  if (tag?.indirectCategories && tag.indirectCategories.length > 0) {
    return tag.indirectCategories[0];
  }
  
  // Try to infer from backlinks
  const backlinks = getAllBacklinks(registry, tagName);
  for (const backlink of backlinks) {
    const backlinkTag = registry.tags.get(resolveAlias(registry, backlink));
    if (backlinkTag?.directCategory) {
      return backlinkTag.directCategory;
    }
    if (backlinkTag?.indirectCategories && backlinkTag.indirectCategories.length > 0) {
      return backlinkTag.indirectCategories[0];
    }
  }
  
  return undefined;
}

/**
 * Validate a tag string against the registry
 */
export function validateTag(registry: TagRegistry, tagString: string): ValidationResult {
  let parsed: ParsedTag;
  
  try {
    parsed = parseTag(tagString);
  } catch (e) {
    return {
      valid: false,
      error: `Parse error: ${(e as Error).message}`,
    };
  }
  
  // Check if main tag exists directly
  if (tagExists(registry, parsed.mainTag)) {
    const resolvedTag = getTag(registry, parsed.mainTag)!;
    return {
      valid: true,
      parsed,
      category: parsed.category || inferCategory(registry, parsed.mainTag),
      validatedBy: resolvedTag.name,
      resolvedBacklinks: getAllBacklinks(registry, parsed.mainTag),
    };
  }
  
  // Check if backlink chain leads to a known tag
  if (parsed.backlinks.length > 0) {
    const firstBacklinks = parsed.backlinks[0].tags;
    for (const backlink of firstBacklinks) {
      if (tagExists(registry, backlink)) {
        // The tag is valid because it extends a known tag
        const allBacklinks: string[] = [];
        for (const bl of parsed.backlinks) {
          allBacklinks.push(...bl.tags);
        }
        
        return {
          valid: true,
          parsed,
          category: parsed.category || inferCategory(registry, backlink),
          validatedBy: backlink,
          resolvedBacklinks: allBacklinks,
        };
      }
    }
    
    // Check deeper in the chain
    for (const bl of parsed.backlinks) {
      for (const backlink of bl.tags) {
        if (tagExists(registry, backlink)) {
          const allBacklinks: string[] = [];
          for (const b of parsed.backlinks) {
            allBacklinks.push(...b.tags);
          }
          
          return {
            valid: true,
            parsed,
            category: parsed.category || inferCategory(registry, backlink),
            validatedBy: backlink,
            resolvedBacklinks: allBacklinks,
          };
        }
      }
    }
  }
  
  return {
    valid: false,
    error: `Tag "${parsed.mainTag}" not found in reference and has no valid backlink chain`,
    parsed,
  };
}

/**
 * Get all possible breadcrumb paths for a tag.
 * Returns an array of paths, where each path is an array of tag names
 * from root to the target tag.
 * 
 * @example
 * // For mypy with backlinks to Python and Statically-Typed:
 * getBreadcrumbPaths(registry, 'mypy')
 * // Returns:
 * // [
 * //   ['Programming Languages', 'Interpreted', 'Python', 'mypy'],
 * //   ['Programming Languages', 'Dynamically-Typed', 'Python', 'mypy'],
 * //   ['Programming Languages', 'Typing', 'Statically-Typed', 'mypy'],
 * // ]
 */
export function getBreadcrumbPaths(
  registry: TagRegistry, 
  tagName: string,
  maxDepth: number = 10
): string[][] {
  const resolved = resolveAlias(registry, tagName);
  const paths: string[][] = [];
  
  function buildPaths(currentTag: string, currentPath: string[], depth: number): void {
    if (depth > maxDepth) return;
    
    const tag = registry.tags.get(currentTag);
    if (!tag) {
      // Tag doesn't exist in registry - this is a leaf/root
      paths.push([...currentPath]);
      return;
    }
    
    if (tag.backlinks.length === 0) {
      // No backlinks - this is a root
      paths.push([...currentPath]);
      return;
    }
    
    // Explore each backlink
    for (const backlink of tag.backlinks) {
      // Prevent cycles
      if (currentPath.includes(backlink)) {
        paths.push([...currentPath]);
        continue;
      }
      buildPaths(backlink, [backlink, ...currentPath], depth + 1);
    }
  }
  
  buildPaths(resolved, [resolved], 0);
  
  // Remove duplicate paths
  const uniquePaths = paths.filter((path, index, arr) => 
    arr.findIndex(p => p.join(' > ') === path.join(' > ')) === index
  );
  
  return uniquePaths;
}

/**
 * Get all edges for building a graph representation
 */
export function getGraphEdges(registry: TagRegistry): Array<{ from: string; to: string }> {
  const edges: Array<{ from: string; to: string }> = [];
  
  for (const [tagName, tag] of registry.tags) {
    for (const frontlink of tag.frontlinks) {
      edges.push({ from: tagName, to: frontlink });
    }
  }
  
  return edges;
}

/**
 * Get subgraph centered on a specific tag
 */
export function getTagSubgraph(
  registry: TagRegistry, 
  centerTag: string, 
  depth: number = 2
): { nodes: string[]; edges: Array<{ from: string; to: string }> } {
  const resolved = resolveAlias(registry, centerTag);
  const nodes = new Set<string>();
  const edges: Array<{ from: string; to: string }> = [];
  
  function explore(tagName: string, currentDepth: number, direction: 'up' | 'down' | 'both') {
    if (currentDepth > depth) {
      return;
    }
    
    // Skip if already visited at this or deeper level
    if (nodes.has(tagName)) {
      return;
    }
    nodes.add(tagName);
    
    const tag = registry.tags.get(tagName);
    if (!tag) return;
    
    if (direction === 'up' || direction === 'both') {
      for (const backlink of tag.backlinks) {
        explore(backlink, currentDepth + 1, 'up');
      }
    }
    
    if (direction === 'down' || direction === 'both') {
      for (const frontlink of tag.frontlinks) {
        explore(frontlink, currentDepth + 1, 'down');
      }
    }
  }
  
  // First: explore and collect all nodes
  explore(resolved, 0, 'both');
  
  // Second: build edges only between nodes in the set
  for (const nodeName of nodes) {
    const tag = registry.tags.get(nodeName);
    if (!tag) continue;
    
    for (const frontlink of tag.frontlinks) {
      if (nodes.has(frontlink)) {
        edges.push({ from: nodeName, to: frontlink });
      }
    }
  }
  
  // Deduplicate edges
  const uniqueEdges = edges.filter((e, i, arr) => 
    arr.findIndex(x => x.from === e.from && x.to === e.to) === i
  );
  
  return {
    nodes: Array.from(nodes),
    edges: uniqueEdges,
  };
}
