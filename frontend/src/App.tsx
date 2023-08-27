import { createMuiTheme, ThemeProvider } from '@material-ui/core';
import React, { useState } from 'react';
import ModalContainer from 'react-modal-promise';
import './App.scss';
import MenuBar from './components/menu';
import Editor from './core/editor';
import { GlobalState, IGlobalState } from './core/store';
import Board from './pages/board';

/**
 * Use default dark theme from Material UI
 */
const darkTheme = createMuiTheme({
  palette: {
    type: 'dark',
  },
});



function App() {

  const [width, setWidth] = useState(50);

  const handleWidthChange = (newWidth:number) => {
    setWidth(newWidth);
  };
  const editor = Editor.getInstance();
  // Global state of the application.
  const [state, setState] = useState<IGlobalState>({
    locked: editor.locked(),
    showingPackage: editor.showingPackage()
  });

  return (
    <div className="app-container">
      <div className="column" style={{ flex: width }}>
      <div style={{ width: '100%' }}>
        <ThemeProvider theme={darkTheme}>
        <div className="App theme-dark">
          <MenuBar editor={editor} />
          {/* Global State */}
          <GlobalState.Provider value={{ state, setState }} >
            <Board editor={editor} />
          </GlobalState.Provider>
        </div>
          <ModalContainer />
        </ThemeProvider> 
        </div> 
      </div>
      <div className="column" style={{ flex: 100 - width }}>
        <iframe src="http://127.0.0.1:8000/exercises/" width="100%" height="100%" title="Facebook"></iframe>
      </div>
    </div>
    
  );
}

export default App;
