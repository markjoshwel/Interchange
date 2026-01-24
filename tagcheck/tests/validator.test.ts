/**
 * Unit tests for the tag validator and search
 */

import { describe, test, expect, beforeAll } from 'bun:test';
import { 
  buildRegistry, 
  findTag, 
  searchTags, 
  getRootTags,
  getBreadcrumbPaths,
  resolveAlias,
  escapeTagName
} from '../src';
import type { TagsToml } from '../src/types';

// Mock registry data
const mockData: TagsToml = {
  interchange: {
    reference: {
      tags: {
        "Programming": [
          "Programming Languages",
          "Programming Languages / Compiled",
          "Programming Languages / Interpreted",
          "Compiled / C++",
          "Compiled / Swift",
          "Interpreted / Python",
          "Interpreted / JavaScript | JS",
          "Scripting / Bash",
        ],
        "Design": [
          "UI Design",
          "UX Design",
        ]
      }
    }
  }
};

let registry: ReturnType<typeof buildRegistry>;

beforeAll(() => {
  registry = buildRegistry(mockData);
});

describe('findTag', () => {
  test('exact match returns exact tag', () => {
    expect(findTag(registry, 'Python')).toBe('Python');
  });

  test('case-insensitive match returns canonical tag', () => {
    expect(findTag(registry, 'python')).toBe('Python');
    expect(findTag(registry, 'PYTHON')).toBe('Python');
  });

  test('exact alias match returns target tag', () => {
    expect(findTag(registry, 'JS')).toBe('JavaScript');
  });

  test('case-insensitive alias match returns target tag', () => {
    expect(findTag(registry, 'js')).toBe('JavaScript');
  });

  test('non-existent tag returns undefined', () => {
    expect(findTag(registry, 'NonExistent')).toBeUndefined();
  });
});

describe('searchTags', () => {
  test('returns matches starting with query', () => {
    const results = searchTags(registry, 'Pro');
    expect(results).toContain('Programming Languages');
  });

  test('returns matches containing query', () => {
    const results = searchTags(registry, 'script');
    expect(results).toContain('JavaScript'); // Contains 'script'
    expect(results).toContain('Scripting'); // Starts with 'Script' (case insensitive)
  });

  test('returns matches for aliases', () => {
    const results = searchTags(registry, 'J');
    expect(results).toContain('JS'); // Alias start with J
    expect(results).toContain('JavaScript'); // Tag starts with J
  });

  test('case insensitive search', () => {
    const results = searchTags(registry, 'swift');
    expect(results).toContain('Swift');
  });
});

describe('getRootTags', () => {
  test('groups root tags by category', () => {
    const roots = getRootTags(registry);
    expect(roots['Programming']).toContain('Programming Languages');
    expect(roots['Design']).toContain('UI Design');
    expect(roots['Design']).toContain('UX Design');
  });
});

describe('getBreadcrumbPaths', () => {
  test('returns path to root', () => {
    const paths = getBreadcrumbPaths(registry, 'Python');
    // Path: Programming Languages -> Interpreted -> Python
    // Note: Our registry builder infers 'Interpreted' has root? No, 'Interpreted' is backlink of 'Python'.
    // 'Interpreted' is backlink of 'Python'.
    // 'Programming Languages' is backlink of 'Interpreted'.
    // So path: Python -> Interpreted -> Programming Languages
    
    // The function returns [root, ..., leaf]
    expect(paths.length).toBeGreaterThan(0);
    const path = paths[0];
    expect(path[path.length - 1]).toBe('Python');
    expect(path[0]).toBe('Programming Languages');
  });

  test('handles aliases in path lookup', () => {
    // Lookup via alias 'JS' shoud return path for 'JavaScript'
    const paths = getBreadcrumbPaths(registry, 'JS');
    expect(paths.length).toBeGreaterThan(0);
    expect(paths[0][paths[0].length - 1]).toBe('JavaScript');
  });
});

describe('resolveAlias', () => {
  test('resolves alias to main tag', () => {
    expect(resolveAlias(registry, 'JS')).toBe('JavaScript');
  });

  test('returns tag parsing if no alias', () => {
    expect(resolveAlias(registry, 'Python')).toBe('Python');
  });
});
