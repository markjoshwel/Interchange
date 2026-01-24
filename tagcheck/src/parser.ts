/**
 * Tag string parser following the pseudogrammar:
 * [ '(' category_name ')' ] [ backlink_tag [ ',' union_backlink_tag_member ] '/' ]* main_tag [ '|' main_tag_alias ]*
 */

import type { ParsedTag, BacklinkNode } from './types';

// Escape placeholders for parsing
const ESCAPED_SLASH = '\u0001';
const ESCAPED_PIPE = '\u0002';
const ESCAPED_COMMA = '\u0003';

/**
 * Replace escaped characters with placeholders
 */
function escapeSpecialChars(input: string): string {
  return input
    .replace(/\\\//g, ESCAPED_SLASH)
    .replace(/\\\|/g, ESCAPED_PIPE)
    .replace(/\\,/g, ESCAPED_COMMA);
}

/**
 * Restore escaped characters from placeholders
 */
function unescapeSpecialChars(input: string): string {
  return input
    .replace(new RegExp(ESCAPED_SLASH, 'g'), '/')
    .replace(new RegExp(ESCAPED_PIPE, 'g'), '|')
    .replace(new RegExp(ESCAPED_COMMA, 'g'), ',');
}

/**
 * Escape special characters in a tag name for use in tag strings.
 * Use this when you have a raw tag name (e.g., "CI/CD") and need to
 * format it for parsing (e.g., "CI\/CD").
 */
export function escapeTagName(tagName: string): string {
  return tagName
    .replace(/\//g, '\\/')
    .replace(/\|/g, '\\|')
    .replace(/,/g, '\\,');
}

/**
 * Parse a tag string into its components
 * 
 * @param input - The tag string to parse
 * @returns ParsedTag structure with all parsed components
 * @throws Error if the input is malformed
 * 
 * @example
 * parseTag('X') // standalone tag
 * parseTag('X / Y') // Y with backlink to X
 * parseTag('X / Y / Z') // Z -> Y -> X chain
 * parseTag('A | B') // A aliased as B
 * parseTag('G, H / I') // I with backlinks to G and H (union)
 * parseTag('(Platform) Windows') // Windows with Platform category override
 * parseTag('CI\\/CD') // literal "CI/CD"
 */
export function parseTag(input: string): ParsedTag {
  const raw = input;
  let working = escapeSpecialChars(input.trim());
  
  if (!working) {
    throw new Error('Empty tag string');
  }

  // Extract category override: (Category) Tag
  let category: string | undefined;
  const categoryMatch = working.match(/^\(([^)]+)\)\s*/);
  if (categoryMatch) {
    category = unescapeSpecialChars(categoryMatch[1].trim());
    working = working.slice(categoryMatch[0].length);
  }

  // Split by / to get path segments
  const segments = working.split(/\s*\/\s*/).map(s => s.trim()).filter(s => s);
  
  if (segments.length === 0) {
    throw new Error('No tag segments found');
  }

  // Last segment is the main tag (with possible aliases)
  const lastSegment = segments.pop()!;
  
  // Parse aliases from main tag: MainTag | Alias1 | Alias2
  const aliasParts = lastSegment.split(/\s*\|\s*/).map(s => unescapeSpecialChars(s.trim())).filter(s => s);
  
  if (aliasParts.length === 0) {
    throw new Error('No main tag found');
  }

  const mainTag = aliasParts[0];
  const aliases = aliasParts.slice(1);

  // Parse backlinks (remaining segments)
  const backlinks: BacklinkNode[] = segments.map(segment => {
    // Check for union: A, B
    const unionParts = segment.split(/\s*,\s*/).map(s => unescapeSpecialChars(s.trim())).filter(s => s);
    
    // Validate no aliases in backlinks
    for (const part of unionParts) {
      if (part.includes('|')) {
        throw new Error(`Aliases not allowed in backlink path: "${part}"`);
      }
    }
    
    return {
      tags: unionParts,
      isUnion: unionParts.length > 1,
    };
  });

  return {
    raw,
    category,
    mainTag,
    aliases,
    backlinks,
  };
}

/**
 * Format a parsed tag back to string form
 */
export function formatTag(parsed: ParsedTag): string {
  const parts: string[] = [];
  
  // Add category override
  if (parsed.category) {
    parts.push(`(${parsed.category})`);
  }
  
  // Add backlinks
  for (const backlink of parsed.backlinks) {
    parts.push(backlink.tags.join(', '));
  }
  
  // Add main tag with aliases
  const mainWithAliases = [parsed.mainTag, ...parsed.aliases].join(' | ');
  parts.push(mainWithAliases);
  
  if (parsed.category) {
    return `${parts[0]} ${parts.slice(1).join(' / ')}`;
  }
  
  return parts.join(' / ');
}

/**
 * Get all tags mentioned in a parsed tag (main tag, aliases, and all backlinks)
 */
export function getAllMentionedTags(parsed: ParsedTag): string[] {
  const tags: string[] = [parsed.mainTag, ...parsed.aliases];
  
  for (const backlink of parsed.backlinks) {
    tags.push(...backlink.tags);
  }
  
  return tags;
}

/**
 * Get the immediate backlinks for a parsed tag (the closest segment)
 */
export function getImmediateBacklinks(parsed: ParsedTag): string[] {
  if (parsed.backlinks.length === 0) {
    return [];
  }
  return parsed.backlinks[parsed.backlinks.length - 1].tags;
}
