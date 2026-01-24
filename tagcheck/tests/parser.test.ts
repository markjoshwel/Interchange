/**
 * Unit tests for the tag parser
 */

import { describe, test, expect } from 'bun:test';
import { parseTag, formatTag, getAllMentionedTags, getImmediateBacklinks } from '../src/parser';

describe('parseTag', () => {
  test('standalone tag', () => {
    const result = parseTag('X');
    expect(result.mainTag).toBe('X');
    expect(result.aliases).toEqual([]);
    expect(result.backlinks).toEqual([]);
    expect(result.category).toBeUndefined();
  });

  test('tag with backlink', () => {
    const result = parseTag('X / Y');
    expect(result.mainTag).toBe('Y');
    expect(result.backlinks).toHaveLength(1);
    expect(result.backlinks[0].tags).toEqual(['X']);
    expect(result.backlinks[0].isUnion).toBe(false);
  });

  test('tag with deep backlink chain', () => {
    const result = parseTag('X / Y / Z');
    expect(result.mainTag).toBe('Z');
    expect(result.backlinks).toHaveLength(2);
    expect(result.backlinks[0].tags).toEqual(['X']);
    expect(result.backlinks[1].tags).toEqual(['Y']);
  });

  test('tag with alias', () => {
    const result = parseTag('A | B');
    expect(result.mainTag).toBe('A');
    expect(result.aliases).toEqual(['B']);
  });

  test('tag with multiple aliases', () => {
    const result = parseTag('A | B | C');
    expect(result.mainTag).toBe('A');
    expect(result.aliases).toEqual(['B', 'C']);
  });

  test('tag with union backlink', () => {
    const result = parseTag('G, H / I');
    expect(result.mainTag).toBe('I');
    expect(result.backlinks).toHaveLength(1);
    expect(result.backlinks[0].tags).toEqual(['G', 'H']);
    expect(result.backlinks[0].isUnion).toBe(true);
  });

  test('tag with category override', () => {
    const result = parseTag('(Platform) Windows');
    expect(result.mainTag).toBe('Windows');
    expect(result.category).toBe('Platform');
    expect(result.backlinks).toEqual([]);
  });

  test('tag with category and backlinks', () => {
    const result = parseTag('(Platform) Hardware / Peripherals');
    expect(result.mainTag).toBe('Peripherals');
    expect(result.category).toBe('Platform');
    expect(result.backlinks).toHaveLength(1);
    expect(result.backlinks[0].tags).toEqual(['Hardware']);
  });

  test('complex example: Q / R, S / T', () => {
    const result = parseTag('Q / R, S / T');
    expect(result.mainTag).toBe('T');
    expect(result.backlinks).toHaveLength(2);
    expect(result.backlinks[0].tags).toEqual(['Q']);
    expect(result.backlinks[1].tags).toEqual(['R', 'S']);
    expect(result.backlinks[1].isUnion).toBe(true);
  });

  test('escaped forward slash', () => {
    const result = parseTag('CI\\/CD');
    expect(result.mainTag).toBe('CI/CD');
    expect(result.backlinks).toEqual([]);
  });

  test('escaped pipe', () => {
    const result = parseTag('Foo \\| Bar');
    expect(result.mainTag).toBe('Foo | Bar');
    expect(result.aliases).toEqual([]);
  });

  test('backlink with escaped slash', () => {
    const result = parseTag('CI\\/CD / GitHub Actions');
    expect(result.mainTag).toBe('GitHub Actions');
    expect(result.backlinks[0].tags).toEqual(['CI/CD']);
  });

  test('alias at end of backlink chain is valid', () => {
    const result = parseTag('J / K / L | M');
    expect(result.mainTag).toBe('L');
    expect(result.aliases).toEqual(['M']);
    expect(result.backlinks).toHaveLength(2);
  });

  test('empty string throws error', () => {
    expect(() => parseTag('')).toThrow('Empty tag string');
  });

  test('whitespace handling', () => {
    const result = parseTag('  X  /  Y  ');
    expect(result.mainTag).toBe('Y');
    expect(result.backlinks[0].tags).toEqual(['X']);
  });
});

describe('formatTag', () => {
  test('formats standalone tag', () => {
    const parsed = parseTag('X');
    expect(formatTag(parsed)).toBe('X');
  });

  test('formats tag with backlink', () => {
    const parsed = parseTag('X / Y');
    expect(formatTag(parsed)).toBe('X / Y');
  });

  test('formats tag with alias', () => {
    const parsed = parseTag('A | B');
    expect(formatTag(parsed)).toBe('A | B');
  });

  test('formats tag with category', () => {
    const parsed = parseTag('(Platform) Windows');
    expect(formatTag(parsed)).toBe('(Platform) Windows');
  });
});

describe('getAllMentionedTags', () => {
  test('standalone tag', () => {
    const parsed = parseTag('X');
    expect(getAllMentionedTags(parsed)).toEqual(['X']);
  });

  test('tag with backlinks and alias', () => {
    const parsed = parseTag('A / B | C');
    const tags = getAllMentionedTags(parsed);
    expect(tags).toContain('B');
    expect(tags).toContain('C');
    expect(tags).toContain('A');
  });
});

describe('getImmediateBacklinks', () => {
  test('no backlinks returns empty', () => {
    const parsed = parseTag('X');
    expect(getImmediateBacklinks(parsed)).toEqual([]);
  });

  test('returns closest backlinks', () => {
    const parsed = parseTag('A / B / C');
    expect(getImmediateBacklinks(parsed)).toEqual(['B']);
  });

  test('handles union backlinks', () => {
    const parsed = parseTag('A / B, C / D');
    expect(getImmediateBacklinks(parsed)).toEqual(['B', 'C']);
  });
});
