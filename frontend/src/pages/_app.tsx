import type { AppProps } from "next/app";
import "../pages/globals.css";
import "../features/flood-mission/floodMission.css";

export default function MyApp({ Component, pageProps }: AppProps) {
  return <Component {...pageProps} />;
}
