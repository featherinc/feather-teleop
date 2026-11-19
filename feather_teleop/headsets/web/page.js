// The page's status list and its problem line.

export function show(field, text) {
  document.getElementById(field).textContent = text;
}

export function problem(text) {
  show("problem", text);
}
