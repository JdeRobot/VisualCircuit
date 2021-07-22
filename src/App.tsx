import React from 'react';
import logo from './logo.svg';
import './App.css';

interface IData {
  profit: number,
  profit_margin: number,
  tax: number
}

function App() {
  const [sale, setSale] = React.useState(0);
  const [cost, setCost] = React.useState(0);
  const [accepted, setAcc] = React.useState(false);
  const [nRV, setNRV] = React.useState(<></>);

  async function handleSubmit(event: React.FormEvent) {
    event.preventDefault();
    let data:IData;
    const res = await fetch("http://localhost:8000/calc", {
      method: "POST",
      headers: {"Content-type": "application/json"},
      body: JSON.stringify({
        "SP": sale,
        "COST": cost,
      }),
    });
    if (res.ok) {
      data = await res.json();
      setNRV(resultView(data));
      console.log(nRV);
      setAcc(true);
    }
  }

  const formView = (
    <form onSubmit={handleSubmit}>
      <label htmlFor="sale">Sale: </label>
      <input className="inputField" name="sale" value={sale} type="Number" onChange={(e) => setSale(Number(e.target.value))} /> <br />
      <label htmlFor="cost">Cost: </label>
      <input className="inputField" name="cost" value={cost} type="Number" onChange={(e) => setCost(Number(e.target.value))} /> <br />
      <input type="submit"/>
    </form>
  );

  const resultView = (data:IData) => (
    <div>
      <p>Profit: {data.profit}</p>
      <p>Profit Margin: {data.profit_margin}</p>
      <p>Tax @ 20%: {data.tax}</p>
      <button onClick={() => {setAcc(false)}}>Back</button>
    </div>
  );

  if (!accepted) {
    return (
      <div className="App">
        <header className="App-header">
          <img src={logo} className="App-logo" alt="logo" />
          {formView}
        </header>
      </div>
    );
  } else {
    return (
      <div className="App">
        <header className="App-header">
          <img src={logo} className="App-logo" alt="logo" />
          {nRV}
        </header>
      </div>
    );
  }
}

export default App;
