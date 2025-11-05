// app/layout.tsx
import "./globals.css";
import { ThemeProvider } from "next-themes";

export const metadata = {
  title: "GCS Dashboard",
  description: "A single-screen dashboard for CanSat telemetry",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body>
        <ThemeProvider attribute="class" defaultTheme="light">
          {children}
        </ThemeProvider>
      </body>
    </html>
  );
}
