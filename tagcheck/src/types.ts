/**
 * Type definitions for the tag parser and validator
 */

/**
 * Represents a single node in the backlink chain.
 * Can be a single tag or a union of multiple tags.
 */
export interface BacklinkNode {
  /** Array of tag names (single element for simple backlink, multiple for union) */
  tags: string[];
  /** Whether this is a union backlink (multiple tags) */
  isUnion: boolean;
}

/**
 * Result of parsing a tag string according to the grammar.
 * Grammar: [ '(' category_name ')' ] [ backlink_tag [ ',' union_backlink_tag_member ] '/' ]* main_tag [ '|' main_tag_alias ]*
 */
export interface ParsedTag {
  /** Original input string */
  raw: string;
  /** Category override from (Category) prefix, if present */
  category?: string;
  /** Primary tag name */
  mainTag: string;
  /** Alternate names for the tag (from | separators) */
  aliases: string[];
  /** Chain of backlinks from left to right (closest to main tag is last) */
  backlinks: BacklinkNode[];
}

/**
 * A fully resolved tag with computed relationships
 */
export interface ResolvedTag {
  /** Primary name of the tag */
  name: string;
  /** Alternate names */
  aliases: string[];
  /** Direct category from (Category) override */
  directCategory?: string;
  /** Inferred categories from backlink chains */
  indirectCategories: string[];
  /** Tags this tag links back to (parents) */
  backlinks: string[];
  /** Tags that link to this tag (children) */
  frontlinks: string[];
}

/**
 * Result of tag validation
 */
export interface ValidationResult {
  /** Whether the tag is valid */
  valid: boolean;
  /** Error message if invalid */
  error?: string;
  /** The parsed tag structure */
  parsed?: ParsedTag;
  /** Inferred category (from reference or override) */
  category?: string;
  /** Reference tag that validates this tag */
  validatedBy?: string;
  /** Full chain of backlinks resolved */
  resolvedBacklinks?: string[];
}

/**
 * Structure of the tags.toml file
 */
export interface TagsToml {
  interchange: {
    reference: {
      tags: Record<string, string[]>;
    };
  };
}

/**
 * Registry of all known tags built from tags.toml
 */
export interface TagRegistry {
  /** Map of tag name -> ResolvedTag */
  tags: Map<string, ResolvedTag>;
  /** Map of alias -> primary tag name */
  aliases: Map<string, string>;
  /** List of all category names */
  categories: string[];
}
