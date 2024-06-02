import React from 'react';
import { render, screen } from '@testing-library/react';
import App from './App';

test('renders App for test', () => {
  render(<App />);
  const linkElement = screen.getByText(/File/i);
  expect(linkElement).toBeInTheDocument();
});
