export interface RegistryBlock {
  id: string;
  name: string;
  author: string;
  version: string;
  description: string;
  url: string;
  category?: string;
  tags?: string[];
}

export interface RegistryResponse {
  blocks: RegistryBlock[];
}

/**
 * Validates if the downloaded JSON represents a structurally valid VisualCircuit block.
 * This is a structural sanity check to prevent the UI from crashing, not a deep code validation.
 * @param blockData The parsed JSON object downloaded from the marketplace
 * @returns boolean True if the block has the required schema
 */
export function validateMarketplaceBlock(blockData: any): boolean {
  if (!blockData || typeof blockData !== 'object') {
    return false;
  }

  // A standard VisualCircuit block should have an 'editor' and 'package' object.
  // This matches the format used in frontend/src/core/editor.ts -> loadProject / addAsBlock
  if (!blockData.editor || typeof blockData.editor !== 'object') {
    console.error("Validation Error: Missing or invalid 'editor' object in block.");
    return false;
  }

  if (!blockData.package || typeof blockData.package !== 'object') {
    console.error("Validation Error: Missing or invalid 'package' object in block.");
    return false;
  }

  // Ensure the package has a name (which is required to display it)
  if (!blockData.package.name || typeof blockData.package.name !== 'string') {
    console.error("Validation Error: Block package is missing a 'name'.");
    return false;
  }

  // If it has dependencies or design, they should be objects
  if (blockData.design && typeof blockData.design !== 'object') {
    console.error("Validation Error: 'design' must be an object if present.");
    return false;
  }

  return true;
}
