/**
 * Tagcheck - Tag validator for Interchange tags.toml
 * 
 * Browser-compatible exports (no Node.js dependencies).
 * For server-side file loading, use loadRegistry from server code directly.
 */

// Parser exports
export { 
  parseTag, 
  formatTag, 
  getAllMentionedTags, 
  getImmediateBacklinks,
  escapeTagName,
} from './parser';

// Validator exports
export { 
  buildRegistry,
  resolveAlias,
  tagExists,
  getTag,
  getAllBacklinks,
  inferCategory,
  validateTag,
  getGraphEdges,
  getTagSubgraph,
  getBreadcrumbPaths,
} from './validator';

// Type exports
export type {
  ParsedTag,
  BacklinkNode,
  ResolvedTag,
  ValidationResult,
  TagsToml,
  TagRegistry,
} from './types';
