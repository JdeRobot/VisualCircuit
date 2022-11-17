import { createMuiTheme, ThemeProvider } from '@material-ui/core';
import React, { useState } from 'react';
import ModalContainer from 'react-modal-promise';
import './App.scss';
import MenuBar from './components/menu';
import Editor from './core/editor';
import { GlobalState, IGlobalState } from './core/store';
import Board from './pages/board';
import {
  BrowserRouter,
  Routes,
  Route,
} from "react-router-dom";
import Display from './pages/display';

/**
 * Use default dark theme from Material UI
 */
const darkTheme = createMuiTheme({
  palette: {
    type: 'dark',
  },
});



function App() {

  const editor = Editor.getInstance();
  // Global state of the application.
  const [state, setState] = useState<IGlobalState>({
    locked: editor.locked(),
    showingPackage: editor.showingPackage()
  });

  return (
    <ThemeProvider theme={darkTheme}>
      <div className="App theme-dark">
        {/* Global State */}
        <GlobalState.Provider value={{ state, setState }} >
        <BrowserRouter>
          <MenuBar editor={editor} />
          <Routes>
          <Route path="/" element = {<Board editor={editor} />} />
          <Route path="/display" element = {<Display />} />
          </Routes>
        </BrowserRouter>
        </GlobalState.Provider>
      </div>
      <ModalContainer />
    </ThemeProvider>
  );
}

export default App;
