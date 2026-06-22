import React, { useEffect, useState } from 'react';
import {
  Box,
  Button,
  CircularProgress,
  Drawer,
  IconButton,
  List,
  ListItem,
  Typography,
  TextField,
  Chip,
  Divider,
  makeStyles,
  Theme,
  createStyles
} from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import { RegistryBlock, RegistryResponse, validateMarketplaceBlock } from '../../core/validator';
import Editor from '../../core/editor';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    drawerPaper: {
      width: 400,
      padding: theme.spacing(2),
      backgroundColor: theme.palette.background.default,
    },
    header: {
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
      marginBottom: theme.spacing(2),
    },
    blockItem: {
      flexDirection: 'column',
      alignItems: 'flex-start',
      padding: theme.spacing(2),
      border: `1px solid ${theme.palette.divider}`,
      borderRadius: theme.shape.borderRadius,
      marginBottom: theme.spacing(1),
    },
    blockHeader: {
      width: '100%',
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
    },
  })
);

interface MarketplacePanelProps {
  open: boolean;
  onClose: () => void;
  editor: Editor;
}

const MarketplacePanel: React.FC<MarketplacePanelProps> = ({ open, onClose, editor }) => {
  const classes = useStyles();
  const [blocks, setBlocks] = useState<RegistryBlock[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState<string>('');

  useEffect(() => {
    if (open && blocks.length === 0) {
      fetchRegistry();
    }
  }, [open]);

  const fetchRegistry = async () => {
    setLoading(true);
    setError(null);
    try {
      const response = await fetch('https://raw.githubusercontent.com/Sarvesh-Mishra1981/VisualCircuit-resources/main/marketplace/registry.json');
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data: RegistryResponse = await response.json();
      setBlocks(data.blocks);
    } catch (err: any) {
      console.error("Failed to fetch registry:", err);
      setError("Failed to load marketplace blocks.");
    } finally {
      setLoading(false);
    }
  };

  const handleInstall = async (block: RegistryBlock) => {
    try {
      const response = await fetch(block.url);
      if (!response.ok) {
        throw new Error(`Could not download block data (Status: ${response.status})`);
      }
      const blockData = await response.json();
      const isValid = validateMarketplaceBlock(blockData);
      
      if (isValid) {
        // Save to Local Storage
        let installedBlocks = [];
        try {
            const stored = localStorage.getItem('vc_marketplace_blocks');
            if (stored) {
                installedBlocks = JSON.parse(stored);
            }
        } catch (e) {
            console.warn("Could not read local storage", e);
        }
        
        // Avoid duplicates by checking package name
        const exists = installedBlocks.find((b: any) => b.package && b.package.name === blockData.package.name);
        if (!exists) {
            installedBlocks.push(blockData);
            localStorage.setItem('vc_marketplace_blocks', JSON.stringify(installedBlocks));
            
            // Dispatch event so the MenuBar updates
            window.dispatchEvent(new Event('vc_marketplace_updated'));

            alert(`Successfully added ${block.name} to your Downloads Menu!`);
        } else {
            alert(`${block.name} is already in your Downloads!`);
        }
      } else {
        alert(`Failed to validate block ${block.name}.`);
      }
    } catch (err: any) {
      console.error("Installation failed:", err);
      alert(`Installation failed: ${err.message}`);
    }
  };

  const filteredBlocks = blocks.filter(block => {
    const query = searchQuery.toLowerCase();
    const matchesName = block.name.toLowerCase().includes(query);
    const matchesDesc = block.description.toLowerCase().includes(query);
    const matchesTags = block.tags?.some(tag => tag.toLowerCase().includes(query));
    return matchesName || matchesDesc || matchesTags;
  });

  const groupedBlocks = filteredBlocks.reduce((acc, block) => {
    const cat = block.category || 'Uncategorized';
    if (!acc[cat]) acc[cat] = [];
    acc[cat].push(block);
    return acc;
  }, {} as Record<string, RegistryBlock[]>);

  return (
    <Drawer
      anchor="left"
      open={open}
      onClose={onClose}
      classes={{ paper: classes.drawerPaper }}
    >
      <div className={classes.header}>
        <Typography variant="h5" style={{ fontWeight: 'bold' }}>Marketplace</Typography>
        <IconButton onClick={onClose}>
          <CloseIcon />
        </IconButton>
      </div>

      <TextField
        variant="outlined"
        fullWidth
        placeholder="Search blocks, tags..."
        value={searchQuery}
        onChange={(e) => setSearchQuery(e.target.value)}
        style={{ marginBottom: 20 }}
      />

      {loading ? (
        <Box display="flex" justifyContent="center" padding={4}>
          <CircularProgress />
        </Box>
      ) : error ? (
        <Typography color="error">{error}</Typography>
      ) : (
        <List>
          {Object.entries(groupedBlocks).map(([category, catBlocks]) => (
            <React.Fragment key={category}>
              <Typography variant="subtitle1" style={{ fontWeight: 'bold', marginTop: 10, color: '#555' }}>
                {category}
              </Typography>
              {catBlocks.map((block) => (
                <ListItem key={block.id} className={classes.blockItem}>
                  <Box width="100%" display="flex" justifyContent="space-between" alignItems="center">
                    <Box>
                      <Typography variant="h6" style={{ fontSize: '1.1rem' }}>{block.name}</Typography>
                      <Typography variant="caption" color="textSecondary">
                        v{block.version} • by {block.author}
                      </Typography>
                    </Box>
                    <Button variant="contained" color="primary" size="small" onClick={() => handleInstall(block)}>
                      Install
                    </Button>
                  </Box>
                  <Typography variant="body2" style={{ marginTop: 8, color: '#666' }}>
                    {block.description}
                  </Typography>
                  {block.tags && block.tags.length > 0 && (
                    <Box display="flex" flexWrap="wrap" mt={1}>
                      {block.tags.map(tag => (
                        <Chip key={tag} label={tag} size="small" style={{ marginRight: 4, marginBottom: 4 }} />
                      ))}
                    </Box>
                  )}
                </ListItem>
              ))}
            </React.Fragment>
          ))}
          {filteredBlocks.length === 0 && (
            <Typography color="textSecondary" style={{ marginTop: 20, textAlign: 'center' }}>
              No blocks found matching "{searchQuery}"
            </Typography>
          )}
        </List>
      )}
    </Drawer>
  );
};

export default MarketplacePanel;
