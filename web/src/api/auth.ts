const API_BASE = "/api/v1";

interface TokenResponse {
  access_token: string;
  refresh_token: string;
  token_type: string;
}

export async function login(email: string, password: string): Promise<TokenResponse> {
  const res = await fetch(`${API_BASE}/auth/login`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ email, password }),
  });
  if (!res.ok) {
    const err = await res.json();
    throw new Error(err.detail || "로그인에 실패했습니다");
  }
  return res.json();
}

export async function signup(username: string, email: string, password: string): Promise<void> {
  const res = await fetch(`${API_BASE}/auth/signup`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ username, email, password }),
  });
  if (!res.ok) {
    const err = await res.json();
    throw new Error(err.detail || "회원가입에 실패했습니다");
  }
}

export async function refreshToken(refresh_token: string): Promise<TokenResponse> {
  const res = await fetch(`${API_BASE}/auth/refresh`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ refresh_token }),
  });
  if (!res.ok) {
    throw new Error("토큰 갱신에 실패했습니다");
  }
  return res.json();
}
