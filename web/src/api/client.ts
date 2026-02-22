import { refreshToken } from "./auth";

export async function authFetch(url: string, options: RequestInit = {}): Promise<Response> {
  const accessToken = localStorage.getItem("access_token");

  const headers = new Headers(options.headers);
  if (accessToken) {
    headers.set("Authorization", `Bearer ${accessToken}`);
  }

  let res = await fetch(url, { ...options, headers });

  if (res.status === 401) {
    const storedRefreshToken = localStorage.getItem("refresh_token");
    if (!storedRefreshToken) {
      localStorage.clear();
      window.location.href = "/login";
      return res;
    }

    try {
      const tokens = await refreshToken(storedRefreshToken);
      localStorage.setItem("access_token", tokens.access_token);
      localStorage.setItem("refresh_token", tokens.refresh_token);

      headers.set("Authorization", `Bearer ${tokens.access_token}`);
      res = await fetch(url, { ...options, headers });
    } catch {
      localStorage.clear();
      window.location.href = "/login";
    }
  }

  return res;
}
